#include "assemble.h"

#include "log.h"
#include "stm32_hal_tim.h"
#include "stm32_hal_dwt.h"

#include <stddef.h>
#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

/* ISR is the sole writer. Aligned uint32_t loads/stores are atomic on M7 */
static volatile uint32_t s_tim6_ticks;
static uint32_t s_consumed_tick;
static uint32_t s_origin_tick;
static uint32_t s_cycle_start_us;
static bool s_cycle_active;
static AssembleSchedulerDiagnostics s_scheduler;

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void tim6_callback(void);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

SystemStatus assemble_tim6_500hz(void) {
    tim_register_callback(&htim6, tim6_callback);
    s_tim6_ticks = 0u;
    assemble_scheduler_reset();
    if(!tim_start()) {
        log_error("TIM6 start failed");
        return SYSTEM_STATUS_ERROR;
    }
    log_info("Tim6 started");
    return SYSTEM_STATUS_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static void tim6_callback(void) {
    ++s_tim6_ticks;
}

void assemble_scheduler_reset(void) {
    s_consumed_tick = s_tim6_ticks;
    s_origin_tick = s_consumed_tick;
    s_cycle_active = false;
    memset(&s_scheduler, 0, sizeof(s_scheduler));
}

bool assemble_scheduler_take_tick(uint32_t* elapsed_ticks) {
    const uint32_t newest_tick = s_tim6_ticks;
    const uint32_t elapsed = newest_tick - s_consumed_tick;
    if(elapsed == 0u) return false;
    s_consumed_tick = newest_tick;
    ++s_scheduler.cycles;
    s_scheduler.missed_ticks += elapsed - 1u;
    if(elapsed_ticks != NULL) *elapsed_ticks = elapsed;
    s_cycle_start_us = dwt_get_us();
    s_cycle_active = true;
    return true;
}

void assemble_scheduler_finish_tick(void) {
    if(!s_cycle_active) return;
    const uint32_t runtime = dwt_get_us() - s_cycle_start_us;
    s_cycle_active = false;
    s_scheduler.last_runtime_us = runtime;
    if(runtime > s_scheduler.max_runtime_us) s_scheduler.max_runtime_us = runtime;
    if(runtime > 2000u) ++s_scheduler.overruns;
}

void assemble_scheduler_get_diagnostics(AssembleSchedulerDiagnostics* diagnostics) {
    if(diagnostics == NULL) return;
    *diagnostics = s_scheduler;
    diagnostics->ticks = s_tim6_ticks - s_origin_tick;
}

void assemble_scheduler_log_diagnostics(void) {
    AssembleSchedulerDiagnostics diagnostics;
    assemble_scheduler_get_diagnostics(&diagnostics);
    log_info("SCHED ticks=%lu cycles=%lu missed=%lu overruns=%lu max_us=%lu last_us=%lu",
             (unsigned long)diagnostics.ticks, (unsigned long)diagnostics.cycles,
             (unsigned long)diagnostics.missed_ticks, (unsigned long)diagnostics.overruns,
             (unsigned long)diagnostics.max_runtime_us, (unsigned long)diagnostics.last_runtime_us);
}
