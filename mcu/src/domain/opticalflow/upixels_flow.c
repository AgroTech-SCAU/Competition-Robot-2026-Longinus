/**
 * @file upixels_flow.c
 * @brief 优象 UP-T1 系列串口帧解析实现（C99，无动态内存）。
 */

#include "upixels_flow.h"

#include <stddef.h>
#include <math.h>
#include <string.h>

#define UPIXELS_HEADER 0xFEu
#define UPIXELS_TAIL 0x55u

#define UPIXELS_FRAME_LEN 14u
#define UPIXELS_EXT_FRAME_LEN 20u
#define PX4_FLOW_FRAME_LEN 52u
#define PX4_DISTANCE_FRAME_LEN 22u

#define PX4_FLOW_MSG_ID 0x6Au
#define PX4_DISTANCE_MSG_ID 0x84u
#define PX4_SYS_ID 0x00u
#define PX4_COMP_ID 0x9Eu

#define PX4_FLOW_CRC_EXTRA 138u
#define PX4_DISTANCE_CRC_EXTRA 85u

/* MAVLink 内部按 IEEE-754 float 解析，这里做编译期检查。 */
typedef char upixels_float_must_be_32bit[(sizeof(float) == 4u) ? 1 : -1];

typedef enum {
    PARSE_NEED_MORE = 0,
    PARSE_INVALID = 1,
    PARSE_OK = 2
} parse_result_t;

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t read_i16(const uint8_t *p)
{
    uint16_t raw = read_u16(p);
    if ((raw & 0x8000u) != 0u) {
        return (int16_t)((int32_t)raw - 65536);
    }
    return (int16_t)raw;
}

static uint32_t read_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static float read_f32(const uint8_t *p)
{
    uint32_t raw = read_u32(p);
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static int32_t round_f32_to_i32(float value)
{
    if (!isfinite(value) || value < -2147483648.0f || value >= 2147483648.0f) {
        return 0;
    }
    return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static uint8_t xor8(const uint8_t *data, uint32_t length)
{
    uint8_t result = 0u;
    while (length-- > 0u) {
        result ^= *data++;
    }
    return result;
}

static uint16_t crc_accumulate(uint16_t crc, uint8_t data)
{
    uint8_t tmp = (uint8_t)(data ^ (uint8_t)(crc & 0xFFu));
    tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
    return (uint16_t)((crc >> 8) ^
                      ((uint16_t)tmp << 8) ^
                      ((uint16_t)tmp << 3) ^
                      ((uint16_t)tmp >> 4));
}

static uint16_t mavlink_crc16(const uint8_t *data, uint32_t length,
                              uint8_t crc_extra)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;

    for (i = 0u; i < length; ++i) {
        crc = crc_accumulate(crc, data[i]);
    }
    crc = crc_accumulate(crc, crc_extra);
    return crc;
}

static bool valid_distance_m(float distance_m)
{
    return (distance_m > 0.0f) &&
           (distance_m < 1000.0f) &&
           (distance_m == distance_m); /* 排除 NaN */
}

static void set_flow_distance(upixels_parser_t *parser, upixels_frame_t *frame,
                              bool valid, float distance_m)
{
    if (valid && valid_distance_m(distance_m)) {
        parser->last_distance_m = distance_m;
        parser->has_last_distance = true;
        parser->last_distance_timestamp_ms = frame->timestamp_ms;
    }
    frame->distance_m = parser->last_distance_m;
    frame->distance_valid = parser->has_last_distance;
}

static void set_distance_event(upixels_parser_t *parser, upixels_frame_t *frame,
                               bool valid, float distance_m)
{
    bool distance_ok = valid && valid_distance_m(distance_m);

    if (distance_ok) {
        parser->last_distance_m = distance_m;
        parser->has_last_distance = true;
        parser->last_distance_timestamp_ms = frame->timestamp_ms;
    }
    frame->distance_m = distance_ok ? distance_m : 0.0f;
    frame->distance_valid = distance_ok;
}

static void parse_upixels_frame(upixels_parser_t *parser, const uint8_t *buf,
                                upixels_protocol_t protocol, uint32_t now_ms,
                                upixels_frame_t *frame)
{
    uint16_t dt_us = read_u16(buf + 6u);
    uint16_t distance_mm = read_u16(buf + 8u);

    memset(frame, 0, sizeof(*frame));
    frame->event = UPIXELS_EVENT_FLOW;
    frame->protocol = protocol;
    frame->timestamp_ms = now_ms;

    frame->flow_x_integral = read_i16(buf + 2u);
    frame->flow_y_integral = read_i16(buf + 4u);
    frame->flow_x_rad = (float)frame->flow_x_integral / 10000.0f;
    frame->flow_y_rad = (float)frame->flow_y_integral / 10000.0f;
    frame->integration_timespan_us = dt_us;
    frame->dt_s = (float)dt_us / 1000000.0f;

    frame->quality = buf[10];
    frame->flow_valid = (buf[10] == UPIXELS_FLOW_VALID);
    frame->tof_confidence = buf[11];

    if (protocol == UPIXELS_PROTOCOL_UPIXELS_EXT) {
        frame->tof_peak = read_u32(buf + 12u);
        frame->tof_noise = read_u16(buf + 16u);
    }

    set_flow_distance(parser, frame,
                      (distance_mm != 0u) && (distance_mm != 0xFFFFu),
                      (float)distance_mm / 1000.0f);
}

static void parse_px4_flow(upixels_parser_t *parser, const uint8_t *buf,
                           uint32_t now_ms, upixels_frame_t *frame)
{
    const uint8_t *p = buf + 6u;
    uint32_t integration_us = read_u32(p + 8u);
    float integrated_x = read_f32(p + 12u);
    float integrated_y = read_f32(p + 16u);
    float distance_m = read_f32(p + 36u);
    uint8_t quality = p[43u];

    memset(frame, 0, sizeof(*frame));
    frame->event = UPIXELS_EVENT_FLOW;
    frame->protocol = UPIXELS_PROTOCOL_PX4;
    frame->timestamp_ms = now_ms;

    frame->flow_x_integral = round_f32_to_i32(integrated_x * 10000.0f);
    frame->flow_y_integral = round_f32_to_i32(integrated_y * 10000.0f);
    frame->flow_x_rad = integrated_x;
    frame->flow_y_rad = integrated_y;
    frame->integration_timespan_us = integration_us;
    frame->dt_s = (float)integration_us / 1000000.0f;
    frame->quality = quality;
    frame->flow_valid = (quality == UPIXELS_FLOW_VALID);

    set_flow_distance(parser, frame, distance_m > 0.0f, distance_m);
}

static void parse_px4_distance(upixels_parser_t *parser, const uint8_t *buf,
                               uint32_t now_ms, upixels_frame_t *frame)
{
    const uint8_t *p = buf + 6u;
    uint16_t min_cm = read_u16(p + 4u);
    uint16_t max_cm = read_u16(p + 6u);
    uint16_t current_cm = read_u16(p + 8u);
    bool valid = (current_cm != 0u) && (current_cm != 0xFFFFu);

    memset(frame, 0, sizeof(*frame));
    frame->event = UPIXELS_EVENT_DISTANCE;
    frame->protocol = UPIXELS_PROTOCOL_PX4;
    frame->timestamp_ms = now_ms;
    frame->distance_min_cm = min_cm;
    frame->distance_max_cm = max_cm;

    set_distance_event(parser, frame, valid, (float)current_cm / 100.0f);
}

static parse_result_t parse_frame(upixels_parser_t *parser, uint32_t now_ms,
                                  upixels_frame_t *frame, uint32_t *consumed)
{
    const uint8_t *buf = parser->buffer;
    uint16_t length = parser->length;

    if (length == 0u) {
        return PARSE_NEED_MORE;
    }
    if (buf[0] != UPIXELS_HEADER) {
        return PARSE_INVALID;
    }
    if (length < 2u) {
        return PARSE_NEED_MORE;
    }

    /* FE 0A：标准 14 字节；也兼容扩展协议误写长度字节 0x0A 的 20 字节帧 */
    if (buf[1] == 0x0Au) {
        if (length >= UPIXELS_FRAME_LEN) {
            if ((buf[13] == UPIXELS_TAIL) &&
                (xor8(buf + 2u, 10u) == buf[12])) {
                parse_upixels_frame(parser, buf,
                                    UPIXELS_PROTOCOL_UPIXELS,
                                    now_ms, frame);
                *consumed = UPIXELS_FRAME_LEN;
                return PARSE_OK;
            }
            if (length >= UPIXELS_EXT_FRAME_LEN) {
                if ((buf[19] == UPIXELS_TAIL) &&
                    (xor8(buf + 2u, 16u) == buf[18])) {
                    parse_upixels_frame(parser, buf,
                                        UPIXELS_PROTOCOL_UPIXELS_EXT,
                                        now_ms, frame);
                    *consumed = UPIXELS_EXT_FRAME_LEN;
                    return PARSE_OK;
                }
                parser->crc_errors++;
                return PARSE_INVALID;
            }
            return PARSE_NEED_MORE;
        }
        return PARSE_NEED_MORE;
    }

    /* FE 10：扩展协议 20 字节 */
    if (buf[1] == 0x10u) {
        if (length < UPIXELS_EXT_FRAME_LEN) {
            return PARSE_NEED_MORE;
        }
        if ((buf[19] != UPIXELS_TAIL) ||
            (xor8(buf + 2u, 16u) != buf[18])) {
            parser->crc_errors++;
            return PARSE_INVALID;
        }
        parse_upixels_frame(parser, buf, UPIXELS_PROTOCOL_UPIXELS_EXT,
                            now_ms, frame);
        *consumed = UPIXELS_EXT_FRAME_LEN;
        return PARSE_OK;
    }

    /* PX4 光流：FE 2C ... 6A */
    if (buf[1] == 0x2Cu) {
        uint16_t crc_calc;
        uint16_t crc_recv;

        if (length < 6u) {
            return PARSE_NEED_MORE;
        }
        if ((buf[3] != PX4_SYS_ID) ||
            (buf[4] != PX4_COMP_ID) ||
            (buf[5] != PX4_FLOW_MSG_ID)) {
            return PARSE_INVALID;
        }
        if (length < PX4_FLOW_FRAME_LEN) {
            return PARSE_NEED_MORE;
        }
        crc_calc = mavlink_crc16(buf + 1u, 49u, PX4_FLOW_CRC_EXTRA);
        crc_recv = read_u16(buf + 50u);
        if (crc_calc != crc_recv) {
            parser->crc_errors++;
            return PARSE_INVALID;
        }
        parse_px4_flow(parser, buf, now_ms, frame);
        *consumed = PX4_FLOW_FRAME_LEN;
        return PARSE_OK;
    }

    /* PX4 距离：FE 0E ... 84 */
    if (buf[1] == 0x0Eu) {
        uint16_t crc_calc;
        uint16_t crc_recv;

        if (length < 6u) {
            return PARSE_NEED_MORE;
        }
        if ((buf[3] != PX4_SYS_ID) ||
            (buf[4] != PX4_COMP_ID) ||
            (buf[5] != PX4_DISTANCE_MSG_ID)) {
            return PARSE_INVALID;
        }
        if (length < PX4_DISTANCE_FRAME_LEN) {
            return PARSE_NEED_MORE;
        }
        crc_calc = mavlink_crc16(buf + 1u, 19u, PX4_DISTANCE_CRC_EXTRA);
        crc_recv = read_u16(buf + 20u);
        if (crc_calc != crc_recv) {
            parser->crc_errors++;
            return PARSE_INVALID;
        }
        parse_px4_distance(parser, buf, now_ms, frame);
        *consumed = PX4_DISTANCE_FRAME_LEN;
        return PARSE_OK;
    }

    return PARSE_INVALID;
}

static bool process_buffer(upixels_parser_t *parser, uint32_t now_ms,
                           upixels_frame_t *frame)
{
    while (parser->length > 0u) {
        uint32_t consumed = 0u;
        parse_result_t result = parse_frame(parser, now_ms, frame, &consumed);

        if (result == PARSE_NEED_MORE) {
            return false;
        }

        if (result == PARSE_INVALID) {
            parser->bytes_discarded++;
            memmove(parser->buffer, parser->buffer + 1u,
                    (size_t)(parser->length - 1u));
            parser->length--;
            continue;
        }

        if ((consumed == 0u) || (consumed > parser->length)) {
            /* 防御性处理：解析器状态异常时丢弃一个字节重新同步 */
            parser->bytes_discarded++;
            memmove(parser->buffer, parser->buffer + 1u,
                    (size_t)(parser->length - 1u));
            parser->length--;
            continue;
        }

        memmove(parser->buffer, parser->buffer + consumed,
                (size_t)(parser->length - consumed));
        parser->length = (uint16_t)(parser->length - consumed);
        parser->frames_ok++;
        return true;
    }

    return false;
}

void upixels_parser_init(upixels_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
}

bool upixels_parser_feed(upixels_parser_t *parser, uint8_t byte,
                         uint32_t now_ms, upixels_frame_t *frame)
{
    if ((parser == NULL) || (frame == NULL)) {
        return false;
    }

    if (parser->length >= (uint16_t)UPIXELS_PARSER_BUFFER_SIZE) {
        memmove(parser->buffer, parser->buffer + 1u,
                (size_t)(UPIXELS_PARSER_BUFFER_SIZE - 1u));
        parser->length = (uint16_t)(UPIXELS_PARSER_BUFFER_SIZE - 1u);
        parser->bytes_discarded++;
    }

    parser->buffer[parser->length++] = byte;
    return process_buffer(parser, now_ms, frame);
}

bool upixels_parser_poll(upixels_parser_t *parser, uint32_t now_ms,
                         upixels_frame_t *frame)
{
    if ((parser == NULL) || (frame == NULL)) {
        return false;
    }
    return process_buffer(parser, now_ms, frame);
}

bool upixels_frame_velocity(const upixels_frame_t *frame, float height_m,
                            float *vx_m_s, float *vy_m_s)
{
    if ((frame == NULL) || (vx_m_s == NULL) || (vy_m_s == NULL)) {
        return false;
    }
    if ((frame->event != UPIXELS_EVENT_FLOW) || (!frame->flow_valid)) {
        return false;
    }
    if (!(frame->dt_s > 0.0f) ||
        !(height_m > 0.0f) ||
        !(height_m < 1000.0f)) {
        return false;
    }
    if (!isfinite(frame->flow_x_rad) || !isfinite(frame->flow_y_rad)) {
        return false; /* NaN */
    }

    *vx_m_s = frame->flow_x_rad * height_m / frame->dt_s;
    *vy_m_s = frame->flow_y_rad * height_m / frame->dt_s;
    return true;
}
