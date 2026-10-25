#ifndef _assemble_h_
#define _assemble_h_

#include <stdbool.h>
#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    SYSTEM_STATUS_OK = 0,
    SYSTEM_STATUS_ERROR,
} SystemStatus;

typedef struct {
    uint32_t ticks;
    uint32_t cycles;
    uint32_t missed_ticks;
    uint32_t overruns;
    uint32_t max_runtime_us;
    uint32_t last_runtime_us;
} AssembleSchedulerDiagnostics;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

SystemStatus assemble_delay(void);
SystemStatus assemble_log(void);
SystemStatus assemble_rgb(void);
SystemStatus assemble_imu(void);
SystemStatus assemble_chassis(void);
SystemStatus assemble_odom(void);
SystemStatus assemble_remote(void);
SystemStatus assemble_tim6_500hz(void);
/** Consume the newest tick once; elapsed_ticks includes skipped periods
 * Call finish after the control cycle. Main-loop only; no backlog replay */
bool assemble_scheduler_take_tick(uint32_t* elapsed_ticks);
void assemble_scheduler_finish_tick(void);
void assemble_scheduler_reset(void);
void assemble_scheduler_get_diagnostics(AssembleSchedulerDiagnostics* diagnostics);
void assemble_scheduler_log_diagnostics(void);

void assemble_chassis_log_diagnostics(void);
void assemble_remote_log_diagnostics(void);

#endif
