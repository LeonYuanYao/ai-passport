#pragma once

// Compact PC -> device usage snapshot. Token counts are stored in units of
// 10,000 tokens: this keeps the 24 hourly buckets at two bytes each while still
// covering 655.35M tokens in one hour. The PC sends the newest 24 local-hour
// buckets and the top three models; the device never parses logs or JSON.
//
// Day wire packet (105 bytes, little-endian):
//   [0]       magic 0x55 ('U')
//   [1]       version 1
//   [2]       local hour (0..23) represented by hourly bucket 0
//   [3]       populated model slots (0..3)
//   [4..7]    total tokens / 10,000 (uint32)
//   [8..55]   24 hourly totals / 10,000 (24 x uint16)
//   [56..103] 3 model slots: 12-byte NUL-padded ASCII name + uint32 total / 10,000
//   [104]     XOR of every preceding byte (XOR of the whole packet == 0)
//
// Week wire packet (85 bytes, little-endian) is separate so version-1 devices
// keep accepting the unchanged day snapshot:
//   [0]       magic 0x57 ('W')
//   [1]       version 1
//   [2]       local weekday (Monday=0) represented by daily bucket 0
//   [3]       populated model slots (0..3)
//   [4..7]    seven-day total tokens / 10,000 (uint32)
//   [8..35]   7 daily totals / 10,000 (7 x uint32)
//   [36..83]  3 model slots: 12-byte NUL-padded ASCII name + uint32 total / 10,000
//   [84]      XOR of every preceding byte (XOR of the whole packet == 0)

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ISLAND_USAGE_MAGIC 0x55
#define ISLAND_USAGE_VERSION 1
#define ISLAND_USAGE_HOUR_COUNT 24
#define ISLAND_USAGE_MODEL_COUNT 3
#define ISLAND_USAGE_MODEL_NAME_LEN 12
#define ISLAND_USAGE_TOTAL_OFFSET 4
#define ISLAND_USAGE_BUCKETS_OFFSET 8
#define ISLAND_USAGE_MODELS_OFFSET 56
#define ISLAND_USAGE_MODEL_SLOT_LEN 16
#define ISLAND_USAGE_PACKET_LEN 105
#define ISLAND_USAGE_WEEK_MAGIC 0x57
#define ISLAND_USAGE_WEEK_VERSION 1
#define ISLAND_USAGE_DAY_COUNT 7
#define ISLAND_USAGE_WEEK_TOTAL_OFFSET 4
#define ISLAND_USAGE_WEEK_DAYS_OFFSET 8
#define ISLAND_USAGE_WEEK_MODELS_OFFSET 36
#define ISLAND_USAGE_WEEK_PACKET_LEN 85

typedef struct {
    char name[ISLAND_USAGE_MODEL_NAME_LEN];
    uint32_t tokens_10k;
} island_usage_model_t;

typedef struct {
    uint8_t start_hour;
    uint8_t model_count;
    uint32_t total_10k;
    uint16_t hourly_10k[ISLAND_USAGE_HOUR_COUNT];
    island_usage_model_t models[ISLAND_USAGE_MODEL_COUNT];
} island_usage_t;

typedef struct {
    uint8_t start_weekday;
    uint8_t model_count;
    uint32_t total_10k;
    uint32_t daily_10k[ISLAND_USAGE_DAY_COUNT];
    island_usage_model_t models[ISLAND_USAGE_MODEL_COUNT];
} island_usage_week_t;

typedef enum {
    ISLAND_USAGE_RANGE_DAY = 0,
    ISLAND_USAGE_RANGE_WEEK,
} island_usage_range_t;

// Parse only a complete, checksummed snapshot. On failure *out is untouched so
// the UI keeps its last good value instead of flashing partial telemetry.
bool island_usage_parse(const uint8_t *buf, size_t len, island_usage_t *out);
bool island_usage_week_parse(const uint8_t *buf, size_t len,
                             island_usage_week_t *out);

island_usage_range_t island_usage_next_range(island_usage_range_t current);

// Scale one hourly value into a chart height. Values above peak saturate and a
// missing/zero peak produces zero, keeping LVGL-specific math out of the parser.
uint8_t island_usage_bar_height(uint32_t value, uint32_t peak,
                                uint8_t max_height);
