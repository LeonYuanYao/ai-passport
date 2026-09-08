#include "island_usage.h"

#include <assert.h>
#include <string.h>

static void put_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = value & 0xff;
    dst[1] = value >> 8;
}

static void put_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = value & 0xff;
    dst[1] = value >> 8;
    dst[2] = value >> 16;
    dst[3] = value >> 24;
}

static void seal(uint8_t packet[ISLAND_USAGE_PACKET_LEN])
{
    packet[ISLAND_USAGE_PACKET_LEN - 1] = 0;
    for (size_t i = 0; i + 1 < ISLAND_USAGE_PACKET_LEN; ++i) {
        packet[ISLAND_USAGE_PACKET_LEN - 1] ^= packet[i];
    }
}

static void seal_week(uint8_t packet[ISLAND_USAGE_WEEK_PACKET_LEN])
{
    packet[ISLAND_USAGE_WEEK_PACKET_LEN - 1] = 0;
    for (size_t i = 0; i + 1 < ISLAND_USAGE_WEEK_PACKET_LEN; ++i) {
        packet[ISLAND_USAGE_WEEK_PACKET_LEN - 1] ^= packet[i];
    }
}

static void make_packet(uint8_t packet[ISLAND_USAGE_PACKET_LEN])
{
    memset(packet, 0, ISLAND_USAGE_PACKET_LEN);
    packet[0] = ISLAND_USAGE_MAGIC;
    packet[1] = ISLAND_USAGE_VERSION;
    packet[2] = 16;  // oldest bucket starts at 16:00 local time
    packet[3] = 3;
    put_u32(packet + ISLAND_USAGE_TOTAL_OFFSET, 64460);  // 644.6M tokens

    put_u16(packet + ISLAND_USAGE_BUCKETS_OFFSET + 0 * 2, 6390);
    put_u16(packet + ISLAND_USAGE_BUCKETS_OFFSET + 12 * 2, 7290);
    put_u16(packet + ISLAND_USAGE_BUCKETS_OFFSET + 23 * 2, 600);

    uint8_t *model = packet + ISLAND_USAGE_MODELS_OFFSET;
    memcpy(model, "GPT-5.6-SOL", 11);
    put_u32(model + ISLAND_USAGE_MODEL_NAME_LEN, 40070);
    model += ISLAND_USAGE_MODEL_SLOT_LEN;
    memcpy(model, "GPT-5.5", 7);
    put_u32(model + ISLAND_USAGE_MODEL_NAME_LEN, 22320);
    model += ISLAND_USAGE_MODEL_SLOT_LEN;
    memcpy(model, "OPUS-5", 6);
    put_u32(model + ISLAND_USAGE_MODEL_NAME_LEN, 2060);
    seal(packet);
}

static void test_parse_snapshot(void)
{
    uint8_t packet[ISLAND_USAGE_PACKET_LEN];
    make_packet(packet);

    island_usage_t usage = {0};
    assert(island_usage_parse(packet, sizeof(packet), &usage));
    assert(usage.start_hour == 16);
    assert(usage.model_count == 3);
    assert(usage.total_10k == 64460);
    assert(usage.hourly_10k[0] == 6390);
    assert(usage.hourly_10k[12] == 7290);
    assert(usage.hourly_10k[23] == 600);
    assert(strcmp(usage.models[0].name, "GPT-5.6-SOL") == 0);
    assert(usage.models[0].tokens_10k == 40070);
    assert(strcmp(usage.models[1].name, "GPT-5.5") == 0);
    assert(usage.models[1].tokens_10k == 22320);
    assert(strcmp(usage.models[2].name, "OPUS-5") == 0);
    assert(usage.models[2].tokens_10k == 2060);
}

static void test_rejects_without_clobbering(void)
{
    uint8_t packet[ISLAND_USAGE_PACKET_LEN];
    make_packet(packet);
    island_usage_t usage = {.total_10k = 1234};

    packet[10] ^= 1;
    assert(!island_usage_parse(packet, sizeof(packet), &usage));
    assert(usage.total_10k == 1234);

    make_packet(packet);
    packet[2] = 24;
    seal(packet);
    assert(!island_usage_parse(packet, sizeof(packet), &usage));

    make_packet(packet);
    packet[3] = ISLAND_USAGE_MODEL_COUNT + 1;
    seal(packet);
    assert(!island_usage_parse(packet, sizeof(packet), &usage));
}

static void test_bar_height(void)
{
    assert(island_usage_bar_height(0, 100, 39) == 0);
    assert(island_usage_bar_height(50, 100, 39) == 20);
    assert(island_usage_bar_height(100, 100, 39) == 39);
    assert(island_usage_bar_height(200, 100, 39) == 39);
    assert(island_usage_bar_height(10, 0, 39) == 0);
}

static void test_parse_week_snapshot(void)
{
    uint8_t packet[ISLAND_USAGE_WEEK_PACKET_LEN] = {0};
    packet[0] = ISLAND_USAGE_WEEK_MAGIC;
    packet[1] = ISLAND_USAGE_WEEK_VERSION;
    packet[2] = 5;  // Saturday, where Monday == 0
    packet[3] = 3;
    put_u32(packet + ISLAND_USAGE_WEEK_TOTAL_OFFSET, 370300);
    put_u32(packet + ISLAND_USAGE_WEEK_DAYS_OFFSET + 0 * 4, 31240);
    put_u32(packet + ISLAND_USAGE_WEEK_DAYS_OFFSET + 4 * 4, 72860);
    put_u32(packet + ISLAND_USAGE_WEEK_DAYS_OFFSET + 6 * 4, 64460);

    uint8_t *model = packet + ISLAND_USAGE_WEEK_MODELS_OFFSET;
    memcpy(model, "GPT-5.6-SOL", 11);
    put_u32(model + ISLAND_USAGE_MODEL_NAME_LEN, 222000);
    model += ISLAND_USAGE_MODEL_SLOT_LEN;
    memcpy(model, "GPT-5.5", 7);
    put_u32(model + ISLAND_USAGE_MODEL_NAME_LEN, 126000);
    model += ISLAND_USAGE_MODEL_SLOT_LEN;
    memcpy(model, "OPUS-5", 6);
    put_u32(model + ISLAND_USAGE_MODEL_NAME_LEN, 22300);
    seal_week(packet);

    island_usage_week_t usage = {0};
    assert(island_usage_week_parse(packet, sizeof(packet), &usage));
    assert(usage.start_weekday == 5);
    assert(usage.model_count == 3);
    assert(usage.total_10k == 370300);
    assert(usage.daily_10k[0] == 31240);
    assert(usage.daily_10k[4] == 72860);
    assert(usage.daily_10k[6] == 64460);
    assert(strcmp(usage.models[0].name, "GPT-5.6-SOL") == 0);
    assert(usage.models[0].tokens_10k == 222000);
    assert(strcmp(usage.models[2].name, "OPUS-5") == 0);
    assert(usage.models[2].tokens_10k == 22300);

    assert(island_usage_next_range(ISLAND_USAGE_RANGE_DAY) ==
           ISLAND_USAGE_RANGE_WEEK);
    assert(island_usage_next_range(ISLAND_USAGE_RANGE_WEEK) ==
           ISLAND_USAGE_RANGE_DAY);
}

int main(void)
{
    test_parse_snapshot();
    test_rejects_without_clobbering();
    test_bar_height();
    test_parse_week_snapshot();
    return 0;
}
