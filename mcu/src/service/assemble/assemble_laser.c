#include "assemble.h"

#include "delay.h"
#include "fdcan.h"
#include "laser_odom.h"
#include "log.h"
#include "stm32_hal_can.h"

#define LASER_CAN_ID 0x601u

/* The ISR owns writes to this mailbox. The service runs in the main loop. */
static volatile LaserOdomSample s_laser_rx;

static void assemble_laser_receive(FDCAN_HandleTypeDef* hcan, const FDCAN_RxHeaderTypeDef* header,
                                   const uint8_t data[8], void* user) {
    (void)hcan;
    (void)user;
    if(header->Identifier != LASER_CAN_ID || header->IdType != FDCAN_STANDARD_ID ||
       header->RxFrameType != FDCAN_DATA_FRAME || header->FDFormat != FDCAN_CLASSIC_CAN ||
       header->DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }

    for(unsigned int beam = 0; beam < LASER_BEAM_COUNT; ++beam) {
        s_laser_rx.distance_mm[beam] = (uint16_t)((uint16_t)data[2u * beam] << 8u) |
                                     (uint16_t)data[2u * beam + 1u];
    }
    s_laser_rx.stamp_ms = HAL_GetTick();
    if(++s_laser_rx.received == 0u) s_laser_rx.received = 1u;
}

SystemStatus assemble_laser(void) {
    FDCAN_FilterTypeDef filter = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0u,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1 = LASER_CAN_ID,
        .FilterID2 = 0x7FFu,
    };
    if(HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK ||
       HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
       can_register_rx_callback(&hfdcan3, assemble_laser_receive, NULL) != STM32_HAL_CAN_OK ||
       can_start(&hfdcan3) != STM32_HAL_CAN_OK) {
        return SYSTEM_STATUS_ERROR;
    }
    return SYSTEM_STATUS_OK;
}

void assemble_laser_process(void) {
    const uint32_t irq = can_critical_enter();
    const LaserOdomSample sample = s_laser_rx;
    can_critical_exit(irq);
    if(sample.received != laser_odom_get_sample()->received) {
        laser_odom_receive(&sample);
    }
}

void assemble_laser_log_diagnostics(void) {
    BspCanDiagnostics bus = { 0 };
    can_refresh_diagnostics(&hfdcan3);
    (void)can_get_diagnostics(&hfdcan3, &bus);
    const LaserOdomSample* sample = laser_odom_get_sample();
    log_info("LASER CAN3 %s id=0x601 frames=%lu mask=0x%X REC=%lu busoff=%u",
             laser_odom_is_online() ? "OK" : (sample->received ? "TIMEOUT" : "WAIT"),
             (unsigned long)sample->received, (unsigned int)sample->valid_mask,
             (unsigned long)bus.rx_error_count, bus.bus_off ? 1u : 0u);
}
