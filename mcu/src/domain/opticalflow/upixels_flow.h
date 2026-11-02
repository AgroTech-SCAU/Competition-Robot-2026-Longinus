/**
 * @file upixels_flow.h
 * @brief 优象 UP-T1 系列光流/TOF 模块的串口帧解析（无 ROS、无动态内存）。
 *
 * 支持协议：
 *   - UPIXELS       : FE 0A + 10 字节 + XOR + 55（14 字节）
 *   - UPIXELS_EXT   : FE 10（或 FE 0A 兼容）+ 16 字节 + XOR + 55（20 字节）
 *   - MAVLink V1 PX4: FE 2C ... 6A（52 字节）+ FE 0E ... 84（22 字节）
 *
 * 使用方法：UART 每收到一个字节调用 upixels_parser_feed()；
 * 返回 true 时 frame 中是一帧完整数据。也可以用 DMA 收完一块后循环调用
 * upixels_parser_poll() 把缓冲区里已完整的帧取完。
 *
 * 所有数据小端；解析器内部有 128 字节缓冲，不使用 malloc。
 */

#ifndef UPIXELS_FLOW_H
#define UPIXELS_FLOW_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UPIXELS_FLOW_VALID 0xF5u

#ifndef UPIXELS_PARSER_BUFFER_SIZE
#define UPIXELS_PARSER_BUFFER_SIZE 128u
#endif

#if (UPIXELS_PARSER_BUFFER_SIZE < 52u)
#error "UPIXELS_PARSER_BUFFER_SIZE 必须 >= 52（PX4 帧长度）"
#endif
#if (UPIXELS_PARSER_BUFFER_SIZE > 65535u)
#error "UPIXELS_PARSER_BUFFER_SIZE 不能超过 65535"
#endif

/** 解析器输出的事件类型。 */
typedef enum {
    UPIXELS_EVENT_NONE = 0,
    UPIXELS_EVENT_FLOW = 1,
    UPIXELS_EVENT_DISTANCE = 2
} upixels_event_t;

/** 当前帧使用的协议。 */
typedef enum {
    UPIXELS_PROTOCOL_UNKNOWN = 0,
    UPIXELS_PROTOCOL_UPIXELS = 1,
    UPIXELS_PROTOCOL_UPIXELS_EXT = 2,
    UPIXELS_PROTOCOL_PX4 = 3
} upixels_protocol_t;

/** 一帧解析结果。未使用字段在每次输出前会被清零。 */
typedef struct {
    upixels_event_t event;
    upixels_protocol_t protocol;
    uint32_t timestamp_ms;          /**< 调用方传入的时间戳 */

    /* 光流 */
    int32_t flow_x_integral;        /**< rad * 10000；PX4 由 integrated_x 反算 */
    int32_t flow_y_integral;
    float flow_x_rad;               /**< 本帧积分角位移 [rad] */
    float flow_y_rad;
    float dt_s;                     /**< 积分时间 [s] */
    uint32_t integration_timespan_us;
    bool flow_valid;                /**< valid/quality == 0xF5 */
    uint8_t quality;

    /* 激光测距：flow 帧里是“本帧测距或最近一次有效测距” */
    float distance_m;
    bool distance_valid;
    uint16_t distance_min_cm;
    uint16_t distance_max_cm;

    /* 优象协议附加信息 */
    uint16_t tof_confidence;
    uint32_t tof_peak;
    uint16_t tof_noise;
} upixels_frame_t;

/** 解析器状态，清零后使用。 */
typedef struct {
    uint8_t buffer[UPIXELS_PARSER_BUFFER_SIZE];
    uint16_t length;

    float last_distance_m;
    bool has_last_distance;
    uint32_t last_distance_timestamp_ms; /**< 最近一次实际有效测距时间，非缓存使用时间 */

    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t bytes_discarded;
} upixels_parser_t;

/** 初始化解析器（memset + 计数器清零）。 */
void upixels_parser_init(upixels_parser_t *parser);

/**
 * @brief 输入一个 UART 字节。
 * @param parser  解析器
 * @param byte    收到的字节
 * @param now_ms  调用方时间戳（例如 HAL_GetTick()）
 * @param frame   输出帧；返回 true 时有效
 * @return 解析出完整帧返回 true
 */
bool upixels_parser_feed(upixels_parser_t *parser, uint8_t byte,
                         uint32_t now_ms, upixels_frame_t *frame);

/**
 * @brief 不输入新字节，尝试从内部缓冲取出下一帧。
 *        适合 UART DMA 一次收到多帧后循环调用，直到返回 false。
 */
bool upixels_parser_poll(upixels_parser_t *parser, uint32_t now_ms,
                         upixels_frame_t *frame);

/**
 * @brief 由光流积分角位移和高度计算传感器坐标系速度。
 * @param frame     光流帧
 * @param height_m  离地高度 [m]
 * @param vx_m_s    输出 X 速度 [m/s]
 * @param vy_m_s    输出 Y 速度 [m/s]
 * @return 数据有效且 dt>0、height>0 时返回 true
 */
bool upixels_frame_velocity(const upixels_frame_t *frame, float height_m,
                            float *vx_m_s, float *vy_m_s);

#ifdef __cplusplus
}
#endif

#endif /* UPIXELS_FLOW_H */
