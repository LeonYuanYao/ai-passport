#include "island_usage.h"

#include <string.h>

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)p[1] << 8;
}

static uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 |
           (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

bool island_usage_parse(const uint8_t *buf, size_t len, island_usage_t *out)
{
    if (buf == NULL || out == NULL || len != ISLAND_USAGE_PACKET_LEN) {
        return false;
    }
    uint8_t fold = 0;
    for (size_t i = 0; i < len; ++i) fold ^= buf[i];
    if (fold != 0 || buf[0] != ISLAND_USAGE_MAGIC ||
        buf[1] != ISLAND_USAGE_VERSION || buf[2] > 23 ||
        buf[3] > ISLAND_USAGE_MODEL_COUNT) {
        return false;
    }

    island_usage_t parsed = {
        .start_hour = buf[2],
        .model_count = buf[3],
        .total_10k = read_u32(buf + ISLAND_USAGE_TOTAL_OFFSET),
    };
    for (size_t i = 0; i < ISLAND_USAGE_HOUR_COUNT; ++i) {
        parsed.hourly_10k[i] = read_u16(
            buf + ISLAND_USAGE_BUCKETS_OFFSET + i * sizeof(uint16_t));
    }
    for (size_t i = 0; i < ISLAND_USAGE_MODEL_COUNT; ++i) {
        const uint8_t *slot = buf + ISLAND_USAGE_MODELS_OFFSET +
                              i * ISLAND_USAGE_MODEL_SLOT_LEN;
        memcpy(parsed.models[i].name, slot, ISLAND_USAGE_MODEL_NAME_LEN);
        parsed.models[i].name[ISLAND_USAGE_MODEL_NAME_LEN - 1] = '\0';
        parsed.models[i].tokens_10k = read_u32(
            slot + ISLAND_USAGE_MODEL_NAME_LEN);
    }
    *out = parsed;
    return true;
}

uint8_t island_usage_bar_height(uint16_t value, uint16_t peak,
                                uint8_t max_height)
{
    if (value == 0 || peak == 0 || max_height == 0) return 0;
    if (value >= peak) return max_height;
    return (uint8_t)(((uint32_t)value * max_height + peak / 2) / peak);
}
