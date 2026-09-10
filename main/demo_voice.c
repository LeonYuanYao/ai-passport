// Path B voice input — the device is a BLE wireless microphone for the PC.
//
// The device has no Wi-Fi in its use environment, so audio streams over a BLE
// GATT service (voice_ble.c) direct to a paired PC running the recv-ble agent.
// 16 kHz mu-law is sent frame by frame — stateless per sample, so a dropped BLE
// notify costs only that frame instead of corrupting the stream (ADPCM would
// drift), while one byte per sample halves the frames per second the link must
// carry. See VOICE_CHUNK_SAMPLES for why the frame RATE is the binding limit.
//
//   DOWN = start / stop the mic
//   OK   = 发送  (stop if recording, then control -> PC injects Enter)
//   UP   = 删除  (control -> PC injects Backspace; held = erase continuously)
//
// Audio never touches the LVGL/button task: a worker owns capture + encode +
// notify. Capture streams in small blocks; no whole-recording buffer.
#include "demo.h"

#include "bsp_audio.h"
#include "bsp_display.h"     // 低功耗模式调背光,见 VOICE_DIM_AFTER_MS
#include "bsp_battery.h"
#include "island_quota.h"
#include "island_usage.h"
#include "ui_pixel.h"
#include "voice_ble.h"
#include "voice_proto.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "demo_voice";

#define VOICE_SAMPLE_RATE 16000
// BLE lets roughly one notify through per connection event, so the NUMBER of
// frames per second — not their size, and not bandwidth — is what the link
// bounds, and a shortfall is a hole in the PC's mic stream that a streaming ASR
// reads as a sentence end.
//
// 480 samples of mu-law is 480 bytes, the same 482-byte packet that 240 samples
// of 16-bit PCM produced, at half the frame rate: 33.3/s instead of 66.7/s. The
// PCM form measured 85-92% delivered when macOS granted a 15 ms connection
// interval and fell to 47-56% — almost exactly half — after a reconnect
// renegotiated it to 30 ms, with the device's own pool_dry counter going from
// ~19 to ~235 per session as unsent notifications filled the mbuf pool. The
// device cannot make macOS grant more events, so it sends fewer, larger-payload
// frames instead. See voice_ulaw_encode for what the compression costs.
//
// Do not raise this to fill the MTU further: 252 samples (506 of 507 available
// bytes) was tried and measured WORSE, 89% -> 69%, because a fuller packet costs
// more of the controller's ACL buffers. Headroom is worth more than the slots.
#define VOICE_CHUNK_SAMPLES 480
#define VOICE_CHUNK_MS (VOICE_CHUNK_SAMPLES * 1000 / VOICE_SAMPLE_RATE)   // 30 ms

// ST_CONNECTING here means "advertising / waiting for the PC to subscribe".
typedef enum { ST_CONNECTING, ST_IDLE, ST_RECORDING, ST_ERROR } voice_state_t;

static lv_obj_t *s_scr;
static lv_obj_t *s_title;
static lv_obj_t *s_link_icon;
static lv_obj_t *s_big;
static lv_obj_t *s_sub;
static lv_obj_t *s_battery;
static lv_obj_t *s_battery_shell;
static lv_obj_t *s_battery_fill;
static lv_obj_t *s_battery_cap;
static lv_obj_t *s_usage_kicker;
static lv_obj_t *s_usage_layer;
static lv_obj_t *s_voice_layer;
static lv_obj_t *s_meter;
static lv_obj_t *s_total;
static lv_obj_t *s_peak;
static lv_obj_t *s_hour_bar[ISLAND_USAGE_HOUR_COUNT];
static lv_obj_t *s_axis[4];
static lv_obj_t *s_model_name[ISLAND_USAGE_MODEL_COUNT];
static lv_obj_t *s_model_value[ISLAND_USAGE_MODEL_COUNT];
static lv_obj_t *s_model_bar[ISLAND_USAGE_MODEL_COUNT];
static lv_obj_t *s_model_dot[ISLAND_USAGE_MODEL_COUNT];
static lv_obj_t *s_range_bg[2];
static lv_obj_t *s_range_label[2];
static lv_obj_t *s_quota_value;
static lv_obj_t *s_quota_bar;
static lv_timer_t *s_timer;

static SemaphoreHandle_t s_lock;
static SemaphoreHandle_t s_worker_done;
static TaskHandle_t s_worker;

static volatile voice_state_t s_state = ST_CONNECTING;
static volatile bool s_want_record;      // UI -> worker: capture on/off
static volatile bool s_closing;
static volatile int s_pending_ctrl;      // UI -> worker: one-shot ctrl code
static volatile unsigned s_elapsed_ms;
// Worker -> render(): capture loudness 0..100. Plain volatile int: a torn read
// costs one frame of the meter, and it must NOT share s_lock with the worker's
// hot path (lock contention there measurably starved the audio stream).
static volatile int s_level;
static unsigned s_lvl_tick;              // worker-only: frames since the last RMS
#define VOICE_SILENCE_LEVEL 10           // below this the smoothed level reads quiet
// The RMS runs every other frame, so the counters below tick in 60 ms units
// regardless of the frame length. Pinning the UNIT rather than the frame count is
// what keeps the silence timeout at 3 s when the frame size changes.
#define VOICE_LVL_EVERY 2                            // 2 * 30 ms = 60 ms
#define VOICE_SILENCE_TICKS 50                       // 50 * 60 ms = 3.0 s ends a take
// One in eight samples, i.e. 60 per frame — the count the meter was tuned against.
// This runs between the capture read and the notify, so it is charged straight to
// the frame budget: sampling every sample once measurably cut delivery 99% -> 95%.
#define VOICE_LVL_STRIDE 8
static bool s_backlog;                   // worker-only: one frame awaiting a retry
static unsigned s_quiet_ticks;           // worker-only: consecutive quiet RMS ticks
static int32_t s_hp_x, s_hp_y;           // worker-only: high-pass filter state
static uint64_t s_read_us, s_send_us, s_retry_us;
// Press-to-first-frame, in microseconds. Both stamps come from esp_timer on this
// device, so this is a single-clock measurement — the PC's clock is never involved
// and there is nothing to reconcile.
static uint64_t s_first_frame_us;
static int64_t s_record_start_us;
static unsigned s_tx_frames;   // audio frames encoded (diagnostic)
static unsigned s_tx_ok;       // frames the BLE stack accepted (diagnostic)
static int s_battery_percent = -1;
static island_quota_t s_quota;
static bool s_have_quota;
static island_usage_t s_usage;
static bool s_have_usage;
static island_usage_week_t s_usage_week;
static bool s_have_usage_week;
static island_usage_range_t s_usage_range;
static bool s_usage_dirty;
static int s_drawn_state = -1;
static int s_drawn_quota = -2;
static int s_drawn_battery = -2;
static unsigned s_battery_poll_ms;

// Integer square root, bit-by-bit restoring. Keeps float sqrt out of the audio
// worker for a value that only drives a 0..100 display.
static uint32_t isqrt64(uint64_t n)
{
    uint64_t rem = 0, root = 0;
    for (int i = 0; i < 32; i++) {
        root <<= 1;
        rem = (rem << 2) | (n >> 62);
        n <<= 2;
        if (root < rem) { rem -= root | 1; root |= 2; }
    }
    return (uint32_t)(root >> 1);
}

static void set_state(voice_state_t st)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_state = st;
    xSemaphoreGive(s_lock);
}

// The NimBLE host task delivers both small quota packets and the larger usage
// snapshot here. Parsing happens before the lock; the critical section only
// copies a validated value so it cannot stall the audio worker.
static void on_telemetry(const uint8_t *data, size_t len)
{
    island_quota_t q;
    island_usage_t usage;
    island_usage_week_t week;
    bool is_quota = island_quota_parse(data, len, &q);
    bool is_usage = !is_quota && island_usage_parse(data, len, &usage);
    bool is_week = !is_quota && !is_usage &&
                   island_usage_week_parse(data, len, &week);
    if ((!is_quota && !is_usage && !is_week) || s_lock == NULL) return;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (is_quota) {
        s_quota = q;
        s_have_quota = true;
    } else if (is_usage) {
        s_usage = usage;
        s_have_usage = true;
        s_usage_dirty = true;
    } else {
        s_usage_week = week;
        s_have_usage_week = true;
        s_usage_dirty = true;
    }
    xSemaphoreGive(s_lock);
}

static void worker_task(void *arg)
{
    (void)arg;
    static int16_t pcm[VOICE_CHUNK_SAMPLES];
    // Encoded frame, and the one frame held for a single retry. Both are mu-law
    // bytes, not samples: keeping the retry copy in encoded form means the retry
    // sends the identical bytes and re-encodes nothing.
    static uint8_t enc[VOICE_CHUNK_SAMPLES];
    static uint8_t backlog[VOICE_CHUNK_SAMPLES];
    uint16_t backlog_seq = 0;                      // valid only while s_backlog
    s_backlog = false;
    bool capturing = false;

    // Install the sink before advertising starts. A previously paired Mac can
    // reconnect and write its initial snapshot immediately after BLE sync.
    voice_ble_set_data_cb(on_telemetry);
    if (voice_ble_start() != ESP_OK) {
        ESP_LOGW(TAG, "BLE start failed");
        set_state(ST_ERROR);
        goto done;
    }
    if (bsp_audio_set_format(VOICE_SAMPLE_RATE, 16, 1) != ESP_OK) {
        ESP_LOGW(TAG, "audio format set failed");
        set_state(ST_ERROR);
        goto done;
    }

    while (!s_closing) {
        bool ready = voice_ble_ready();
        // One-shot control (send/delete) — only meaningful once connected.
        int ctrl = s_pending_ctrl;
        // Hold a SEND back while capture is still running: OK during recording means
        // stop-then-send, and Enter must not reach 豆包 before the STOP notify does.
        bool send_deferred = (ctrl == VOICE_CTRL_SEND && capturing);
        if (send_deferred) ctrl = 0;
        if (ctrl != 0 && ready && voice_ctrl_valid(ctrl)) {
            // Clear only on success: a control code that could not be sent is
            // retried next iteration rather than dropped. The stop key felt dead
            // because this cleared unconditionally and the send often failed with
            // the pool exhausted by audio.
            if (voice_ble_send_ctrl((uint8_t)ctrl)) s_pending_ctrl = 0;
        } else if (ctrl != 0 && !voice_ctrl_valid(ctrl)) {
            s_pending_ctrl = 0;             // malformed; drop it
        }

        if (s_want_record && ready && !capturing) {
            capturing = true; s_elapsed_ms = 0; s_level = 0; s_lvl_tick = 0;
            s_quiet_ticks = 0;
            bsp_audio_reset_rx_overflows();   // per-session count, see the STOP log
            voice_ble_reset_audio_seq();      // so the PC reports per-session loss
            s_hp_x = s_hp_y = 0;              // no filter ring-in from the last take
            voice_ble_reset_audio_stats();
            s_read_us = s_send_us = s_retry_us = 0;
            s_first_frame_us = 0;
            s_record_start_us = esp_timer_get_time();
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_tx_frames = 0;
            s_tx_ok = 0;
            xSemaphoreGive(s_lock);
            ESP_LOGI(TAG, "recording START");
            voice_ble_send_ctrl(VOICE_CTRL_START);
            set_state(ST_RECORDING);
        } else if (!s_want_record && capturing) {
            // Stop only when the USER stops. Earlier this also stopped on
            // !ready, but voice_ble_ready() just reports the connection handle
            // and can read false for a moment while the controller is busy — a
            // single blip then knocked the state back to idle, so the recording
            // screen (timer, level, "录音中") was never visible even though audio
            // kept streaming. A genuinely dropped link is handled by the
            // reconnect path below.
            capturing = false; s_level = 0;
            unsigned ovf = (unsigned)bsp_audio_rx_overflows();
            voice_ble_log_audio_stats();
            // The overflow count is in I2S DMA descriptors, and one descriptor is
            // dma_frame_num samples (bsp_audio.c) — NOT one BLE frame. The two
            // coincided while the BLE frame was also 240 samples, so this divided
            // by the right thing by accident; at 480 it would report double. This
            // is the one metric that separates a device-side drop from a link-side
            // one, which is exactly the ambiguity that misdirected debugging
            // before, so it divides by the descriptor length explicitly.
            #define I2S_DESC_MS (240 * 1000 / VOICE_SAMPLE_RATE)   // bsp dma_frame_num
            ESP_LOGI(TAG, "recording STOP %u.%us i2s_ovf=%u (%u%% of frames)",
                     s_elapsed_ms / 1000, s_elapsed_ms % 1000 / 100, ovf,
                     s_elapsed_ms ? ovf * 100 / (s_elapsed_ms / I2S_DESC_MS) : 0);
            if (ready) voice_ble_send_ctrl(VOICE_CTRL_STOP);
            if (ready) {
                // Where the loop's time went, little-endian, milliseconds. The PC
                // prints it with its own figures, which is the only way to tell a
                // device-side shortfall from a delivery one — and needing USB serial
                // to read it meant it was unavailable exactly when the device was in
                // real use.
                unsigned att = 0, acc = 0, af = 0, pf = 0, nf = 0;
                int lrc = 0;
                unsigned ovsz = 0, mtu = 0, msys = 0;
                voice_ble_audio_stats(&att, &acc, &af, &pf, &nf, &lrc,
                                      &ovsz, &mtu, &msys);
                uint16_t st16[14] = {
                    (uint16_t)(ovf > 0xFFFF ? 0xFFFF : ovf),
                    (uint16_t)(s_first_frame_us / 1000),
                    (uint16_t)(s_read_us / 1000),
                    (uint16_t)(s_send_us / 1000),
                    (uint16_t)(s_retry_us / 1000),
                    (uint16_t)(att > 0xFFFF ? 0xFFFF : att),
                    (uint16_t)(acc > 0xFFFF ? 0xFFFF : acc),
                    (uint16_t)(af > 0xFFFF ? 0xFFFF : af),
                    (uint16_t)(nf > 0xFFFF ? 0xFFFF : nf),
                    (uint16_t)(lrc < 0 ? (unsigned)(-lrc) | 0x8000 : lrc),
                    (uint16_t)(ovsz > 0xFFFF ? 0xFFFF : ovsz),
                    (uint16_t)mtu,
                    // Appended, not inserted: an older agent unpacks the first 12
                    // and ignores the tail. append_fail is the SAME dry pool as
                    // alloc_fail caught one step later, and it was the larger half
                    // of the loss while going entirely unreported.
                    (uint16_t)(pf > 0xFFFF ? 0xFFFF : pf),
                    (uint16_t)(msys > 0xFFFF ? 0xFFFF : msys),
                };
                uint8_t buf[1 + sizeof(st16)];
                buf[0] = VOICE_CTRL_STATS;
                memcpy(buf + 1, st16, sizeof(st16));
                if (!voice_ble_send_ctrl_buf(buf, sizeof(buf))) {
                    ESP_LOGW(TAG, "stats frame not sent");
                }
            }
            set_state(ready ? ST_IDLE : ST_CONNECTING);
        } else if (!capturing) {
            set_state(ready ? ST_IDLE : ST_CONNECTING);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Preserve one rejected frame, but retry it before capturing another one.
        // This deliberately applies BLE backpressure to capture rather than using a queue.
        // One retry, then let it go. Two extremes were both worse than this middle:
        // holding a frame and blocking until the pool freed a buffer starved I2S and
        // cost runs of frames to ISR overwrite (retry 8.7 s of a 9.4 s take), while
        // dropping every refused frame outright pushed measured loss from ~0 to
        // 12-25% and the user reported recognition getting worse. So keep one frame
        // for one attempt on the next pass — that covers a momentarily dry pool,
        // which is the common case — and discard it if the second attempt fails
        // rather than waiting. Leaving headroom beats filling every slot.
        if (s_backlog) {
            (void)voice_ble_send_audio(backlog, sizeof(backlog), backlog_seq);
            s_backlog = false;          // sent or not, this frame's turn is over
        }
        int64_t t0 = esp_timer_get_time();
        if (bsp_audio_read(pcm, sizeof(pcm)) != ESP_OK) {
            s_read_us += esp_timer_get_time() - t0;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        s_read_us += esp_timer_get_time() - t0;
        s_elapsed_ms += VOICE_CHUNK_MS;
        // Send mu-law, not raw PCM: still stateless per sample, so a dropped BLE
        // notify costs only that frame instead of corrupting the stream (ADPCM
        // would drift), but at one byte per sample instead of two — which is what
        // halves the frame rate the link has to carry.
        // High-pass the frame in place, one pole at about 90 Hz. The measured
        // spectrum had 11.5% of its energy below 80 Hz and only 36% in the
        // 300-3400 Hz speech band: handling noise and body rumble that carry no
        // speech, consume the headroom that then clips, and skew the ASR's features.
        // Moving the corner to 150 Hz was tried and reverted: sub-300 Hz energy
        // measured HIGHER afterwards (61.9% -> 72.3%), which contradicts the theory,
        // so the change was not justified whatever the explanation. Measure with
        // tools/analyse_take.py before touching this again — and take several
        // recordings, since the level varies more between takes than between builds.
        // y[n] = a*(y[n-1] + x[n] - x[n-1]), a = 1 - 2*pi*fc/fs, fixed point.
        // Q15 coefficient 0.9647 for fc = 90 Hz at 16 kHz.
        for (int i = 0; i < VOICE_CHUNK_SAMPLES; i++) {
            int32_t x = pcm[i];
            int32_t y = (int32_t)(((int64_t)31610 * (s_hp_y + x - s_hp_x)) >> 15);
            s_hp_x = x;
            s_hp_y = y;
            pcm[i] = (int16_t)(y > 32767 ? 32767 : y < -32768 ? -32768 : y);
        }

        // Loudness for the on-screen meter: RMS of the frame, scaled so ordinary
        // speech lands mid-range, then smoothed with a fast attack and slow
        // release so a syllable shows immediately but the bar settles instead of
        // flickering.
        // Every 4th frame only, and over a 1-in-4 sample stride: this runs in the
        // audio worker between the capture read and the BLE notify, so the work is
        // charged directly against the frame budget. Doing it per frame over every
        // sample measurably cut the delivered rate (99% -> 95%). 60 ms updates and
        // 60 samples are ample for a 12-cell bar behind a 100 ms render tick.
        if (++s_lvl_tick >= VOICE_LVL_EVERY) {
            s_lvl_tick = 0;
            uint32_t acc = 0;
            for (int i = 0; i < VOICE_CHUNK_SAMPLES; i += VOICE_LVL_STRIDE) {
                int32_t v = pcm[i] >> 4;      // keep the accumulator in 32 bits
                acc += (uint32_t)(v * v);
            }
            int lvl = (int)(isqrt64(acc / (VOICE_CHUNK_SAMPLES / VOICE_LVL_STRIDE)) * 16 / 40);
            if (lvl > 100) lvl = 100;
            int prev = s_level;
            s_level = lvl > prev ? lvl : prev - (prev - lvl) / 2;

            // Auto-stop on sustained silence, so a take ends by itself when the
            // user stops talking. The level is computed every VOICE_LVL_EVERY
            // frames, which is fixed at 60 ms, so the counter ticks in 60 ms units
            // whatever the frame length. 3 s of quiet: past a pause for thought,
            // short enough not to feel like a hang. Earlier values of 2.4 s clipped
            // people mid-sentence. DOWN stops immediately, and any speech resets
            // the count.
            if (lvl < VOICE_SILENCE_LEVEL) {
                if (++s_quiet_ticks >= VOICE_SILENCE_TICKS) s_want_record = false;
            } else {
                s_quiet_ticks = 0;
            }
        }

        uint16_t seq = voice_ble_next_audio_seq();
        // Encode after the high-pass and the RMS, so both still see linear samples:
        // the filter needs true arithmetic and the meter's thresholds were tuned on
        // PCM. mu-law is per-sample, so this is a flat 480-iteration pass with no
        // state carried between frames.
        for (int i = 0; i < VOICE_CHUNK_SAMPLES; i++) enc[i] = voice_ulaw_encode(pcm[i]);
        t0 = esp_timer_get_time();
        bool ok = voice_ble_send_audio(enc, sizeof(enc), seq);
        if (ok && s_first_frame_us == 0) {
            s_first_frame_us = esp_timer_get_time() - s_record_start_us;
        }
        s_send_us += esp_timer_get_time() - t0;
        if (!ok) {
            memcpy(backlog, enc, sizeof(enc));
            backlog_seq = seq;
            s_backlog = true;           // one retry at the top of the next pass
        }
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_tx_frames++;
        if (ok) s_tx_ok++;
        xSemaphoreGive(s_lock);
    }

done:
    voice_ble_set_data_cb(NULL);
    voice_ble_stop();
    s_worker = NULL;
    xSemaphoreGive(s_worker_done);
    vTaskDelete(NULL);
}

// Low-power mode, entered when the PC link has been down for a while. A 520 mAh
// battery behind a 240x320 backlight at 100% does not last, and while
// disconnected — in a pocket, or on the desk with the agent not running — nothing
// on the screen is worth that current.
//
// Only the backlight and the render cadence change. The BLE stack keeps
// advertising and the audio worker is untouched, so a reconnect and a key press
// behave exactly as before; this must not become a state the device can get stuck
// in.
#define VOICE_DIM_AFTER_MS 20000     // link down this long -> screen off
#define VOICE_DIM_PERCENT  0         // fully off. The panel itself stays on, so a
                                     // key press brings the image straight back with
                                     // nothing to redraw or re-init.
#define VOICE_BRIGHT       50        // normal brightness. 100% was never needed —
                                     // this is a 240x320 panel read at arm's length
                                     // indoors, and the backlight is the largest
                                     // draw on a 520 mAh battery.
#define VOICE_TICK_MS      100       // render period, awake
#define VOICE_TICK_DIM_MS  1000      // render period with the backlight off. Nothing
                                     // is visible, so this only has to be often
                                     // enough to notice the link coming back — the
                                     // agent's own reconnect takes longer than this.
static bool s_dimmed;
static unsigned s_idle_ms;           // render-only: how long nothing has happened

// Which states let the screen go dark. Everything except recording: waiting for
// the agent, connected-and-idle on a desk, and a failed init all show a picture
// that does not change.
static inline bool state_dims(voice_state_t st) { return st != ST_RECORDING; }

// Any key press wakes the screen, whatever else that key does. Called from on_key
// before the key is dispatched, so waking never costs the key its own action.
void demo_voice_wake(void)
{
    s_idle_ms = 0;
    if (!s_dimmed) return;
    s_dimmed = false;
    bsp_display_backlight(VOICE_BRIGHT);
    if (s_timer != NULL) lv_timer_set_period(s_timer, VOICE_TICK_MS);
}

// The chosen Type A dashboard keeps the prototype's dark instrument-panel
// vocabulary while retaining the product palette in its four data accents.
#define DASH_VOID     0x050606
#define DASH_SURFACE  0x0B0D0D
#define DASH_LINE     0x292D2C
#define DASH_PAPER    0xF2F1EB
#define DASH_MUTED    0x777C79
#define DASH_VOICE    0xFF9B73
#define DASH_SIGNAL   0x76D8A0
#define DASH_QUOTA    0xF3C36B
#define DASH_BLUE     0x79A8E8
#define DASH_BATTERY  0xA6AAA8
#define QUOTA_TRACK_WIDTH 112
#define HEADER_RIGHT_MARGIN 12
#define HEADER_LINK_GAP 4
#define HEADER_BATTERY_TEXT_GAP 3
#define BATTERY_INNER_WIDTH 16

static lv_obj_t *dashboard_block(lv_obj_t *parent, int x, int y, int w, int h,
                                 uint32_t color, int radius)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    return obj;
}

static lv_obj_t *dashboard_label(lv_obj_t *parent, const char *text, int x, int y,
                                 const lv_font_t *font, uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    return label;
}

static void refresh_header_battery(void)
{
    int shown = s_battery_percent > 100 ? 100 : s_battery_percent;
    if (shown == s_drawn_battery) return;

    if (shown < 0) lv_label_set_text(s_battery, "--%");
    else lv_label_set_text_fmt(s_battery, "%d%%", shown);

    uint32_t color = DASH_BATTERY;
    int fill_width = 0;
    if (shown >= 0) {
        if (shown <= 20) color = UI_RED;
        else if (shown <= 40) color = DASH_QUOTA;
        fill_width = (shown * BATTERY_INNER_WIDTH + 99) / 100;
        if (fill_width > BATTERY_INNER_WIDTH) fill_width = BATTERY_INNER_WIDTH;
    }
    lv_obj_set_width(s_battery_fill, fill_width);
    lv_obj_set_style_bg_color(s_battery_fill, lv_color_hex(color), 0);
    lv_obj_set_style_border_color(s_battery_shell, lv_color_hex(color), 0);
    lv_obj_set_style_bg_color(s_battery_cap, lv_color_hex(color), 0);
    lv_obj_set_style_text_color(s_battery, lv_color_hex(color), 0);

    // Right-align the complete status cluster. The percentage is auto-sized, so
    // chaining align_to() keeps 9%, 82%, and 100% equally compact. The four-pixel
    // Bluetooth gap is intentional; two pixels looked visually fused on-panel.
    lv_obj_align(s_battery_cap, LV_ALIGN_TOP_RIGHT, -HEADER_RIGHT_MARGIN, 15);
    lv_obj_align_to(s_battery_shell, s_battery_cap, LV_ALIGN_OUT_LEFT_MID, -1, 0);
    lv_obj_align_to(s_battery, s_battery_shell, LV_ALIGN_OUT_LEFT_MID,
                    -HEADER_BATTERY_TEXT_GAP, 0);
    lv_obj_align_to(s_link_icon, s_battery, LV_ALIGN_OUT_LEFT_MID,
                    -HEADER_LINK_GAP, 0);
    s_drawn_battery = shown;
}

static void set_visible(lv_obj_t *obj, bool visible)
{
    if (visible) lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static void format_tokens(char *dst, size_t size, uint32_t tokens_10k)
{
    // Promote before rounding would display 1000.0M; match the HTML preview.
    if (tokens_10k >= 99995) {
        uint32_t hundredths = tokens_10k / 1000 + (tokens_10k % 1000 >= 500);
        snprintf(dst, size, "%u.%02uB", (unsigned)(hundredths / 100),
                 (unsigned)(hundredths % 100));
    } else if (tokens_10k >= 100) {
        uint32_t tenths = (tokens_10k + 5) / 10;
        snprintf(dst, size, "%u.%uM", (unsigned)(tenths / 10),
                 (unsigned)(tenths % 10));
    } else if (tokens_10k > 0) {
        snprintf(dst, size, "%uK", (unsigned)(tokens_10k * 10));
    } else {
        snprintf(dst, size, "--");
    }
}

static void refresh_range_tabs(island_usage_range_t range)
{
    for (size_t i = 0; i < 2; ++i) {
        bool active = i == (size_t)range;
        lv_obj_set_style_bg_color(s_range_bg[i],
                                  lv_color_hex(active ? DASH_LINE : DASH_SURFACE), 0);
        lv_obj_set_style_text_color(s_range_label[i],
                                    lv_color_hex(active ? DASH_PAPER : DASH_MUTED), 0);
    }
}

// Usage objects are restyled only when a new five-minute PC snapshot arrives
// or the user changes range. They never enter the 10 Hz recording redraw path.
static void refresh_usage(const island_usage_t *usage, bool have_usage,
                          const island_usage_week_t *week, bool have_week,
                          island_usage_range_t range)
{
    static const uint32_t colors[ISLAND_USAGE_MODEL_COUNT] = {
        DASH_VOICE, DASH_QUOTA, DASH_SIGNAL,
    };
    static const char *const weekday[ISLAND_USAGE_DAY_COUNT] = {
        "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN",
    };
    char text[24];
    uint32_t peak = 0;
    bool weekly = range == ISLAND_USAGE_RANGE_WEEK;
    bool have_selected = weekly ? have_week : have_usage;
    uint32_t total = weekly ? week->total_10k : usage->total_10k;
    uint8_t model_count = weekly ? week->model_count : usage->model_count;
    const island_usage_model_t *models = weekly ? week->models : usage->models;

    refresh_range_tabs(range);

    if (have_selected) {
        format_tokens(text, sizeof(text), total);
        lv_label_set_text(s_total, text);
        size_t bucket_count = weekly ? ISLAND_USAGE_DAY_COUNT : ISLAND_USAGE_HOUR_COUNT;
        for (size_t i = 0; i < bucket_count; ++i) {
            uint32_t value = weekly ? week->daily_10k[i] : usage->hourly_10k[i];
            if (value > peak) peak = value;
        }
        char peak_text[16];
        format_tokens(peak_text, sizeof(peak_text), peak);
        snprintf(text, sizeof(text), "PEAK %s", peak_text);
        lv_label_set_text(s_peak, text);
    } else {
        lv_label_set_text(s_total, "--");
        lv_label_set_text(s_peak, "PC DATA --");
    }

    for (size_t i = 0; i < ISLAND_USAGE_HOUR_COUNT; ++i) {
        bool visible = !weekly || i < ISLAND_USAGE_DAY_COUNT;
        uint32_t value = weekly && i < ISLAND_USAGE_DAY_COUNT
            ? week->daily_10k[i] : usage->hourly_10k[i];
        int height = have_selected && visible
            ? island_usage_bar_height(value, peak, 43) : 0;
        set_visible(s_hour_bar[i], visible);
        lv_obj_set_x(s_hour_bar[i], weekly ? 10 + (int)i * 28 : 10 + (int)i * 8);
        lv_obj_set_width(s_hour_bar[i], weekly ? 23 : 5);
        lv_obj_set_y(s_hour_bar[i], 58 + 43 - height);
        lv_obj_set_height(s_hour_bar[i], height);
    }

    for (size_t i = 0; i < 4; ++i) {
        if (weekly) {
            unsigned day = (week->start_weekday + i * 2) % ISLAND_USAGE_DAY_COUNT;
            snprintf(text, sizeof(text), "%s", weekday[day]);
        } else {
            unsigned hour = have_usage ? (usage->start_hour + i * 6) % 24 : i * 6;
            snprintf(text, sizeof(text), "%02u", hour);
        }
        lv_label_set_text(s_axis[i], text);
    }

    for (size_t i = 0; i < ISLAND_USAGE_MODEL_COUNT; ++i) {
        bool present = have_selected && i < model_count;
        lv_label_set_text(s_model_name[i], present ? models[i].name : "--");
        format_tokens(text, sizeof(text), present ? models[i].tokens_10k : 0);
        lv_label_set_text(s_model_value[i], text);
        uint32_t width = 0;
        if (present && total > 0) {
            width = (uint32_t)((uint64_t)models[i].tokens_10k * 174 / total);
            if (width > 174) width = 174;
            if (width == 0 && models[i].tokens_10k > 0) width = 2;
        }
        lv_obj_set_width(s_model_bar[i], width);
        lv_obj_set_style_bg_color(s_model_dot[i], lv_color_hex(colors[i]), 0);
        lv_obj_set_style_bg_color(s_model_bar[i], lv_color_hex(colors[i]), 0);
    }
}

static void render(lv_timer_t *t)
{
    (void)t;
    // With the backlight off, the only thing this tick is for is noticing the link
    // come back. Everything below draws or measures for a screen nobody can see, so
    // take the state and leave: no battery ADC conversion, no snprintf, no label
    // updates, and above all no LVGL invalidation — a redraw costs SPI flushes out
    // of the audio worker's budget on this single core.
    if (s_dimmed) {
        if (!state_dims(s_state)) demo_voice_wake();
        return;
    }
    // Battery changes slowly. Polling the blocking I2C gauge at the old 10 Hz
    // render rate bought no visible freshness and stole time from BLE audio.
    s_battery_poll_ms += VOICE_TICK_MS;
    if (s_battery_percent < 0 || s_battery_poll_ms >= 5000) {
        int bp = bsp_battery_soc();
        if (bp >= 0) s_battery_percent = bp;
        s_battery_poll_ms = 0;
    }

    voice_state_t st;
    unsigned ms;
    unsigned tx, ok;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    st = s_state;
    ms = s_elapsed_ms;
    tx = s_tx_frames;
    ok = s_tx_ok;
    bool have_q = s_have_quota;
    island_quota_t q = s_quota;
    bool have_usage = s_have_usage;
    bool have_week = s_have_usage_week;
    bool usage_dirty = s_usage_dirty;
    island_usage_t usage = s_usage;
    island_usage_week_t week = s_usage_week;
    s_usage_dirty = false;
    xSemaphoreGive(s_lock);
    int lvl = s_level;          // plain volatile: deliberately not under s_lock,
                                // which the audio worker takes on its hot path

    if (usage_dirty) {
        refresh_usage(&usage, have_usage, &week, have_week, s_usage_range);
    }

    // Dim on a sustained idle; come straight back the moment anything happens.
    // ST_RECORDING never dims — the timer and level meter are the whole point of
    // that screen. ST_IDLE does: connected to the agent and sitting on a desk is
    // the normal all-day state, and nothing on that screen moves. So does
    // ST_ERROR, which otherwise sits lit until the battery is flat.
    //
    // The dwell is counted in render ticks rather than from a timestamp so it does
    // not need a clock. Every tick reaching here is an awake tick — a dimmed one
    // returned at the top of render() — so VOICE_TICK_MS is the only period that
    // can be added, and the sum stops growing for good once the backlight goes
    // off. A blip that clears within the dwell costs nothing, which is why the
    // link symbol reading false for a moment (see the note in the worker) cannot
    // flicker the backlight.
    if (state_dims(st)) {
        s_idle_ms += VOICE_TICK_MS;
        if (s_idle_ms >= VOICE_DIM_AFTER_MS) {
            s_dimmed = true;
            bsp_display_backlight(VOICE_DIM_PERCENT);
            if (s_timer != NULL) lv_timer_set_period(s_timer, VOICE_TICK_DIM_MS);
        }
    } else {
        demo_voice_wake();      // recording: full brightness, dwell reset
    }

    refresh_header_battery();

    bool linked = st == ST_IDLE || st == ST_RECORDING;
    lv_obj_set_style_text_color(s_link_icon,
                                lv_color_hex(linked ? DASH_SIGNAL : DASH_MUTED), 0);
    lv_obj_set_style_text_opa(s_title,
        st == ST_CONNECTING ? LV_OPA_50 : LV_OPA_COVER, 0);

    // Only Codex's seven-day remaining quota is part of this dashboard. Keep the
    // numeric value beside its track: the bar gives an at-a-glance warning while
    // the number remains useful near the limit. An unavailable value is an empty
    // track plus "--", not a fabricated zero. Restyle only when telemetry changes
    // so the 10 Hz recording render path does not invalidate these widgets.
    int remaining = have_q ? q.codex_remaining_pct : -1;
    if (remaining != s_drawn_quota) {
        if (remaining < 0) {
            lv_label_set_text(s_quota_value, "--");
            lv_obj_set_width(s_quota_bar, 0);
        } else {
            lv_label_set_text_fmt(s_quota_value, "%d%%", remaining);
            int width = (remaining * QUOTA_TRACK_WIDTH + 50) / 100;
            if (width == 0 && remaining > 0) width = 1;
            lv_obj_set_width(s_quota_bar, width);
        }
        s_drawn_quota = remaining;
    }

    if (s_drawn_state != (int)st) {
        bool dashboard = st == ST_IDLE;
        set_visible(s_usage_layer, dashboard);
        set_visible(s_voice_layer, !dashboard);
        s_drawn_state = st;
    }

    switch (st) {
    case ST_CONNECTING:
        lv_label_set_text(s_big, "CONNECTING");
        lv_label_set_text(s_sub, "WAITING FOR MAC");
        break;
    case ST_IDLE:
        break;
    case ST_RECORDING: {
        char time_text[16];
        snprintf(time_text, sizeof(time_text), "%02u:%02u.%u",
                 ms / 60000, ms / 1000 % 60, ms % 1000 / 100);
        lv_label_set_text(s_big, time_text);

        bool healthy = tx == 0 || ok * 100 >= tx * 95;
        lv_label_set_text(s_sub, healthy ? "LISTENING" : "WEAK BLE LINK");
        lv_obj_set_style_text_color(s_sub, lv_color_hex(
            healthy ? DASH_MUTED : DASH_QUOTA), 0);

        // One ASCII label, not 14 individually animated LVGL bars. The latter was
        // measured to starve audio on this single-core/no-PSRAM target.
        int bars = lvl * 14 / 100;
        char meter[15];
        for (int i = 0; i < 14; ++i) meter[i] = i < bars ? '=' : '.';
        meter[14] = '\0';
        lv_label_set_text(s_meter, meter);
        break;
    }
    case ST_ERROR:
        lv_label_set_text(s_big, "ERROR");
        lv_label_set_text(s_sub, "BLE / AUDIO UNAVAILABLE");
        lv_obj_set_style_text_color(s_sub, lv_color_hex(UI_RED), 0);
        break;
    }
}

void demo_voice_enter(void)
{
    s_state = ST_CONNECTING;
    // Enter awake: this screen is reached at boot and on the way back from Feishu,
    // and inheriting a dimmed backlight from a previous visit would look like a
    // fault. s_timer is created below at VOICE_TICK_MS to match.
    s_dimmed = false;
    s_idle_ms = 0;
    bsp_display_backlight(VOICE_BRIGHT);
    s_want_record = false;
    s_closing = false;
    s_pending_ctrl = 0;
    s_battery_percent = -1;
    s_battery_poll_ms = 0;
    s_have_quota = false;
    s_have_usage = false;
    s_have_usage_week = false;
    s_usage_range = ISLAND_USAGE_RANGE_DAY;
    s_usage_dirty = false;
    s_drawn_state = -1;
    s_drawn_quota = -2;
    s_drawn_battery = -2;
    memset(&s_usage, 0, sizeof(s_usage));
    memset(&s_usage_week, 0, sizeof(s_usage_week));
    s_lock = xSemaphoreCreateMutex();
    s_worker_done = xSemaphoreCreateBinary();

    s_scr = lv_obj_create(NULL);
    lv_obj_remove_flag(s_scr, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(s_scr, lv_color_hex(DASH_VOID), 0);
    lv_obj_set_style_border_width(s_scr, 0, 0);
    lv_obj_set_style_pad_all(s_scr, 0, 0);

    s_title = dashboard_label(s_scr, "AI PASSPORT", 12, 10,
                              &lv_font_montserrat_14, DASH_PAPER);
    s_battery = dashboard_label(s_scr, "--%", 0, 0,
                                &lv_font_montserrat_14, DASH_MUTED);
    s_battery_shell = dashboard_block(s_scr, 0, 0, 18, 10,
                                      DASH_VOID, 3);
    lv_obj_set_style_bg_opa(s_battery_shell, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_battery_shell, 1, 0);
    lv_obj_set_style_border_color(s_battery_shell,
                                  lv_color_hex(DASH_BATTERY), 0);
    s_battery_fill = dashboard_block(s_battery_shell, 1, 1, 0, 8,
                                     DASH_BATTERY, 2);
    s_battery_cap = dashboard_block(s_scr, 0, 0, 2, 4,
                                    DASH_BATTERY, 1);
    s_link_icon = dashboard_label(s_scr, LV_SYMBOL_BLUETOOTH, 0, 0,
                                  &lv_font_montserrat_14, DASH_MUTED);
    refresh_header_battery();
    dashboard_block(s_scr, 12, 34, 216, 1, DASH_LINE, 0);

    lv_obj_t *primary = dashboard_block(s_scr, 12, 42, 216, 125,
                                        DASH_SURFACE, 9);
    lv_obj_set_style_border_width(primary, 1, 0);
    lv_obj_set_style_border_color(primary, lv_color_hex(DASH_LINE), 0);

    s_usage_layer = dashboard_block(primary, 1, 1, 214, 123,
                                    DASH_SURFACE, 8);
    s_usage_kicker = dashboard_label(s_usage_layer, "TOKENS / SYNC", 10, 7,
                                     &lv_font_montserrat_14, DASH_MUTED);
    static const char *const range_text[2] = {"1D", "7D"};
    for (size_t i = 0; i < 2; ++i) {
        s_range_bg[i] = dashboard_block(s_usage_layer, 158 + (int)i * 25,
                                        5, 23, 18, DASH_SURFACE, 5);
        s_range_label[i] = dashboard_label(s_range_bg[i], range_text[i], 4, 1,
                                           &lv_font_montserrat_14, DASH_MUTED);
    }
    s_total = dashboard_label(s_usage_layer, "--", 10, 29,
                              &lv_font_montserrat_20, DASH_PAPER);
    s_peak = dashboard_label(s_usage_layer, "PC DATA --", 117, 32,
                             &lv_font_montserrat_14, DASH_VOICE);
    lv_obj_set_width(s_peak, 87);
    lv_obj_set_style_text_align(s_peak, LV_TEXT_ALIGN_RIGHT, 0);

    // Twenty-four plain objects are cheaper than enabling LVGL's chart widget
    // and give each local-hour bucket an exact pixel column on this 240 px UI.
    for (size_t i = 0; i < ISLAND_USAGE_HOUR_COUNT; ++i) {
        s_hour_bar[i] = dashboard_block(s_usage_layer, 10 + (int)i * 8,
                                        101, 5, 0, DASH_BLUE, 2);
    }

    static const int axis_x[4] = {10, 69, 128, 188};
    for (size_t i = 0; i < 4; ++i) {
        s_axis[i] = dashboard_label(s_usage_layer, "--", axis_x[i], 104,
                                    &lv_font_montserrat_14, DASH_MUTED);
    }

    s_voice_layer = dashboard_block(primary, 1, 1, 214, 123,
                                    DASH_SURFACE, 8);
    dashboard_label(s_voice_layer, "VOICE INPUT / RIGHT CMD", 10, 7,
                    &lv_font_montserrat_14, DASH_VOICE);
    s_big = dashboard_label(s_voice_layer, "CONNECTING", 10, 34,
                            &lv_font_montserrat_20, DASH_PAPER);
    s_sub = dashboard_label(s_voice_layer, "WAITING FOR MAC", 10, 61,
                            &lv_font_montserrat_14, DASH_MUTED);
    s_meter = dashboard_label(s_voice_layer, "..............", 10, 91,
                              &lv_font_montserrat_14, DASH_VOICE);

    lv_obj_t *models = dashboard_block(s_scr, 12, 174, 216, 96,
                                       DASH_SURFACE, 8);
    lv_obj_set_style_border_width(models, 1, 0);
    lv_obj_set_style_border_color(models, lv_color_hex(DASH_LINE), 0);
    static const uint32_t model_colors[ISLAND_USAGE_MODEL_COUNT] = {
        DASH_VOICE, DASH_QUOTA, DASH_SIGNAL,
    };
    for (size_t i = 0; i < ISLAND_USAGE_MODEL_COUNT; ++i) {
        int y = 5 + (int)i * 30;
        s_model_dot[i] = dashboard_block(models, 10, y + 4, 7, 7,
                                         model_colors[i], 2);
        s_model_name[i] = dashboard_label(models, "--", 25, y,
                                          &lv_font_montserrat_14, DASH_PAPER);
        lv_obj_set_width(s_model_name[i], 115);
        lv_label_set_long_mode(s_model_name[i], LV_LABEL_LONG_CLIP);
        s_model_value[i] = dashboard_label(models, "--", 142, y,
                                           &lv_font_montserrat_14, DASH_PAPER);
        lv_obj_set_width(s_model_value[i], 64);
        lv_obj_set_style_text_align(s_model_value[i], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_t *track = dashboard_block(models, 25, y + 19, 174, 3,
                                          0x202322, 2);
        s_model_bar[i] = dashboard_block(track, 0, 0, 0, 3,
                                         model_colors[i], 2);
    }

    dashboard_label(s_scr, "CODEX", 14, 282,
                    &lv_font_montserrat_14, DASH_MUTED);
    lv_obj_t *quota_track = dashboard_block(s_scr, 72, 289,
                                            QUOTA_TRACK_WIDTH, 4, 0x202322, 2);
    s_quota_bar = dashboard_block(quota_track, 0, 0, 0, 4, DASH_BLUE, 2);
    s_quota_value = dashboard_label(s_scr, "--", 190, 282,
                                    &lv_font_montserrat_14, DASH_PAPER);
    lv_obj_set_width(s_quota_value, 38);
    lv_obj_set_style_text_align(s_quota_value, LV_TEXT_ALIGN_RIGHT, 0);
    set_visible(s_usage_layer, false);
    set_visible(s_voice_layer, true);
    refresh_usage(&s_usage, false, &s_usage_week, false, s_usage_range);

    lv_screen_load(s_scr);

    if (s_lock == NULL || s_worker_done == NULL) {
        lv_label_set_text(s_big, "MEMORY ERROR");
        lv_label_set_text(s_sub, "RESTART DEVICE");
        return;
    }
    s_timer = lv_timer_create(render, VOICE_TICK_MS, NULL);
    if (xTaskCreate(worker_task, "voice", 6144, NULL, 6, &s_worker) != pdPASS) {
        s_state = ST_ERROR;
    }
}

void demo_voice_exit(void)
{
    s_closing = true;
    s_want_record = false;
    // Hand the next screen a bright backlight. Leaving it dimmed would make
    // onboarding look broken, and that screen has no dim logic of its own.
    s_dimmed = false;
    bsp_display_backlight(VOICE_BRIGHT);
    if (s_worker != NULL && s_worker_done != NULL) {
        xSemaphoreTake(s_worker_done, pdMS_TO_TICKS(2000));
    }
    if (s_timer != NULL) { lv_timer_delete(s_timer); s_timer = NULL; }
    if (s_scr != NULL) { lv_obj_delete(s_scr); s_scr = NULL; }
    if (s_worker_done != NULL) { vSemaphoreDelete(s_worker_done); s_worker_done = NULL; }
    if (s_lock != NULL) { vSemaphoreDelete(s_lock); s_lock = NULL; }
}

void demo_voice_key(bsp_btn_t btn, bsp_btn_ev_t ev)
{
    voice_state_t st = s_state;
    if (st == ST_CONNECTING || st == ST_ERROR) return;

    // Layout follows the physical keys: DOWN (middle) toggles recording, OK
    // (bottom) sends, UP deletes.
    //
    // Toggle and delete fire on PRESS, not CLICK. BSP_BTN_CLICK maps to the
    // button component's BUTTON_SINGLE_CLICK, which is not emitted until the
    // finger LIFTS and the ~180 ms double-click discrimination window expires, so
    // recording started well after the press and the key felt unresponsive.
    // BSP_BTN_PRESS is BUTTON_PRESS_DOWN: the instant contact is made. Both
    // actions are recoverable (press again / say it again), so waiting to learn
    // whether a second click follows buys nothing.
    if (btn == BSP_BTN_DOWN) {
        if (ev == BSP_BTN_PRESS) s_want_record = !s_want_record;
        return;
    }
    if (btn == BSP_BTN_UP) {
        // Short press deletes one utterance; holding erases continuously until the
        // finger lifts. The long-press arrives as a separate event after PRESS has
        // already queued a single delete, so the erase supersedes it — one extra
        // backspace before a hold is harmless, and this keeps the short press
        // instant instead of waiting to rule out a long press.
        //
        // s_erasing is what keeps a short press from sending a stray ERASE_END:
        // RELEASE now fires for every press, not just held ones, and an unpaired
        // END would overwrite the DELETE still waiting in the one-deep ctrl slot.
        static bool s_erasing;
        if (ev == BSP_BTN_PRESS)   s_pending_ctrl = VOICE_CTRL_DELETE;
        if (ev == BSP_BTN_LONG)  { s_pending_ctrl = VOICE_CTRL_ERASE_BEGIN; s_erasing = true; }
        if (ev == BSP_BTN_RELEASE && s_erasing) {
            s_pending_ctrl = VOICE_CTRL_ERASE_END;
            s_erasing = false;
        }
        return;
    }
    // OK reserves double-click for the dashboard's 1D/7D switch. Consequently a
    // single send must use CLICK and wait for the button component's ~180 ms
    // double-click discrimination window. DOWN recording and UP deletion remain
    // immediate PRESS actions. main.c still owns OK's long-hold exit.
    if (ev == BSP_BTN_DOUBLE && st == ST_IDLE) {
        s_usage_range = island_usage_next_range(s_usage_range);
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_usage_dirty = true;
        xSemaphoreGive(s_lock);
        return;
    }
    if (ev == BSP_BTN_CLICK ||
        (ev == BSP_BTN_DOUBLE && st == ST_RECORDING)) {
        // While recording, OK means "I am done — send it": stop capture and send in
        // one gesture, rather than making the user stop with DOWN and then send.
        // A double-click during capture is treated as send, never as a range change.
        // The worker sees s_want_record go false, emits STOP, and then finds the
        // pending SEND, so the PC receives them in that order and 豆包 has finished
        // the utterance before Enter arrives.
        if (st == ST_RECORDING) s_want_record = false;
        s_pending_ctrl = VOICE_CTRL_SEND;
    }
}
