#include "assemble.h"

#include "remote.h"
#include "log.h"
#include "stm32_hal_uart.h"

// ! ========================= 变 量 声 明 ========================= ! //

static FsIa10b s_receiver = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static bool remote_start_receive(void* context, uint8_t* data, uint16_t len);
static bool remote_abort_receive(void* context);
static uint32_t remote_now_ms(void* context);
static void remote_rx_complete_callback(void);
static void remote_rx_error_callback(void);

static const FsIa10bPortOps remote_ops = {
    .start_receive = remote_start_receive,
    .abort_receive = remote_abort_receive,
    .now_ms = remote_now_ms,
    .critical_enter = uart_critical_enter,
    .critical_exit = uart_critical_exit,
};

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_remote(void) {
    const FsIa10bConfig config = {
        .ops = &remote_ops,
        .context = &huart5,
        .frame_timeout_ms = 100u,
        .restart_interval_ms = 200u,
    };
    FsIa10bStatus status;
    uint32_t irq_state;

    log_info("REMOTE init begin");
    irq_state = uart_critical_enter(NULL);
    uart_register_rx_complete_callback(&huart5, remote_rx_complete_callback);
    uart_register_error_callback(&huart5, remote_rx_error_callback);
    status = fs_ia10b_init(&s_receiver, &config);
    if(status == FS_IA10B_STATUS_OK) {
        remote_set_receiver(&s_receiver);
    }
    uart_critical_exit(NULL, irq_state);
    if(status != FS_IA10B_STATUS_OK) {
        log_error("REMOTE init failed: status=%u", (unsigned int)status);
        return SYSTEM_STATUS_ERROR;
    }
    log_info("REMOTE init done");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static bool remote_start_receive(void* context, uint8_t* data, uint16_t len) {
    return uart_receive_it((UART_HandleTypeDef*)context, data, len);
}

static bool remote_abort_receive(void* context) {
    return uart_abort_receive((UART_HandleTypeDef*)context);
}

static uint32_t remote_now_ms(void* context) {
    (void)context;
    return HAL_GetTick();
}

static void remote_rx_complete_callback(void) {
    /* Port failures remain visible through restart_pending and is_online */
    (void)fs_ia10b_on_rx_complete(&s_receiver);
}

static void remote_rx_error_callback(void) {
    (void)fs_ia10b_on_rx_error(&s_receiver);
}

void assemble_remote_log_diagnostics(void) {
    UartRxDiagnostics uart = { 0 };
    FsIa10bData receiver = { 0 };
    if(!uart_get_rx_diagnostics(&huart5, &uart)) return;
    (void)fs_ia10b_get_data(&s_receiver, &receiver);
    log_info("RC bytes=%lu uart_errs=%lu last_err=%lx frames=%lu bad=%lu online=%u",
        (unsigned long)uart.received_bytes, (unsigned long)uart.error_interrupts,
        (unsigned long)uart.last_error, (unsigned long)receiver.frame_count,
        (unsigned long)receiver.error_count, remote_is_online(100u) ? 1u : 0u);
}
