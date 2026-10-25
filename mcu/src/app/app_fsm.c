/**
 * @file app_fsm.c
 * @brief MCU 系统级状态机实现
 */

#include "app_fsm.h"

#include "hfsm/hfsm.h"

#include <string.h>

// ! ========================= 类 型 声 明 ========================= ! //

typedef struct {
    Hfsm fsm;
    HfsmState* idle;
    HfsmState* manual;
    HfsmState* fault;
    HfsmState* estop;
    AppFsmStateId current_state;
    AppFault fault_info;
    bool fault_latched;
} AppFsmContext;

// ! ========================= 变 量 声 明 ========================= ! //

static AppFsmContext s_app_fsm = { 0 };

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static AppFsmContext* app_fsm_context_from_machine(HfsmMachine* machine);
static bool app_fsm_post_high_priority(AppFsmEventId event_id);
static HfsmResult app_fsm_handle_idle(HfsmMachine* machine, const HfsmEvent* event);
static HfsmResult app_fsm_handle_manual(HfsmMachine* machine, const HfsmEvent* event);
static HfsmResult app_fsm_handle_fault(HfsmMachine* machine, const HfsmEvent* event);
static HfsmResult app_fsm_handle_estop(HfsmMachine* machine, const HfsmEvent* event);
static void app_fsm_on_idle_entry(HfsmMachine* machine);
static void app_fsm_on_manual_entry(HfsmMachine* machine);
static void app_fsm_on_fault_entry(HfsmMachine* machine);
static void app_fsm_on_estop_entry(HfsmMachine* machine);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

void app_fsm_init(void) {
    memset(&s_app_fsm, 0, sizeof(s_app_fsm));

    (void)hfsm.init(&s_app_fsm.fsm, &s_app_fsm);

    s_app_fsm.idle = hfsm.add_state(&s_app_fsm.fsm, "Idle");
    s_app_fsm.manual = hfsm.add_state(&s_app_fsm.fsm, "Manual");
    s_app_fsm.fault = hfsm.add_state(&s_app_fsm.fsm, "Fault");
    s_app_fsm.estop = hfsm.add_state(&s_app_fsm.fsm, "EStop");

    (void)hfsm.set_handle(s_app_fsm.idle, app_fsm_handle_idle);
    (void)hfsm.set_handle(s_app_fsm.manual, app_fsm_handle_manual);
    (void)hfsm.set_handle(s_app_fsm.fault, app_fsm_handle_fault);
    (void)hfsm.set_handle(s_app_fsm.estop, app_fsm_handle_estop);

    (void)hfsm.set_entry(s_app_fsm.idle, app_fsm_on_idle_entry);
    (void)hfsm.set_entry(s_app_fsm.manual, app_fsm_on_manual_entry);
    (void)hfsm.set_entry(s_app_fsm.fault, app_fsm_on_fault_entry);
    (void)hfsm.set_entry(s_app_fsm.estop, app_fsm_on_estop_entry);

    (void)hfsm.set_initial(&s_app_fsm.fsm, s_app_fsm.idle);
    (void)hfsm.start(&s_app_fsm.fsm);
}

void app_fsm_process(void) {
    (void)hfsm.process_all(&s_app_fsm.fsm);
}

bool app_fsm_post(AppFsmEventId event_id) {
    return hfsm.post(&s_app_fsm.fsm, (HfsmEventId)event_id, NULL) == HFSM_STATUS_OK;
}

bool app_fsm_raise_fault(const AppFault* fault) {
    if(fault == NULL) {
        return false;
    }

    s_app_fsm.fault_info = *fault;
    s_app_fsm.fault_latched = true;
    return app_fsm_post_high_priority(APP_FSM_EVENT_FAULT);
}

bool app_fsm_request_estop(void) {
    return app_fsm_post_high_priority(APP_FSM_EVENT_ESTOP);
}

bool app_fsm_clear_fault(void) {
    if(s_app_fsm.current_state != APP_FSM_STATE_FAULT || !s_app_fsm.fault_latched ||
       s_app_fsm.fault_info.level != APP_FAULT_LEVEL_RECOVERABLE) {
        return false;
    }

    return app_fsm_post(APP_FSM_EVENT_CLEAR_FAULT);
}

AppFsmStateId app_fsm_get_state(void) {
    return s_app_fsm.current_state;
}

const AppFault* app_fsm_get_fault(void) {
    return &s_app_fsm.fault_info;
}

bool app_fsm_has_fault(void) {
    return s_app_fsm.fault_latched;
}

const char* app_fsm_state_str(AppFsmStateId state) {
    switch(state) {
        case APP_FSM_STATE_IDLE:
            return "Idle";
        case APP_FSM_STATE_MANUAL:
            return "Manual";
        case APP_FSM_STATE_FAULT:
            return "Fault";
        case APP_FSM_STATE_ESTOP:
            return "EStop";
        default:
            return "Unknown";
    }
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static AppFsmContext* app_fsm_context_from_machine(HfsmMachine* machine) {
    return (AppFsmContext*)hfsm.machine.context(machine);
}

static bool app_fsm_post_high_priority(AppFsmEventId event_id) {
    (void)hfsm.clear(&s_app_fsm.fsm);
    return app_fsm_post(event_id);
}

static HfsmResult app_fsm_handle_idle(HfsmMachine* machine, const HfsmEvent* event) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context == NULL || event == NULL) {
        return hfsm.res.ignore();
    }

    switch((AppFsmEventId)event->id) {
        case APP_FSM_EVENT_SWITCH_TO_MANUAL:
            return hfsm.res.transition(context->manual);

        case APP_FSM_EVENT_STOP:
            return hfsm.res.handled();

        case APP_FSM_EVENT_FAULT:
            return hfsm.res.transition(context->fault);

        case APP_FSM_EVENT_ESTOP:
            return hfsm.res.transition(context->estop);

        default:
            return hfsm.res.ignore();
    }
}

static HfsmResult app_fsm_handle_manual(HfsmMachine* machine, const HfsmEvent* event) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context == NULL || event == NULL) {
        return hfsm.res.ignore();
    }

    switch((AppFsmEventId)event->id) {

        case APP_FSM_EVENT_STOP:
            return hfsm.res.transition(context->idle);

        case APP_FSM_EVENT_FAULT:
            return hfsm.res.transition(context->fault);

        case APP_FSM_EVENT_ESTOP:
            return hfsm.res.transition(context->estop);

        default:
            return hfsm.res.ignore();
    }
}

static HfsmResult app_fsm_handle_fault(HfsmMachine* machine, const HfsmEvent* event) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context == NULL || event == NULL) {
        return hfsm.res.ignore();
    }

    switch((AppFsmEventId)event->id) {
        case APP_FSM_EVENT_CLEAR_FAULT:
            memset(&context->fault_info, 0, sizeof(context->fault_info));
            context->fault_latched = false;
            return hfsm.res.transition(context->idle);

        case APP_FSM_EVENT_ESTOP:
            return hfsm.res.transition(context->estop);

        case APP_FSM_EVENT_FAULT:
            return hfsm.res.transition(context->fault);

        default:
            return hfsm.res.ignore();
    }
}

static HfsmResult app_fsm_handle_estop(HfsmMachine* machine, const HfsmEvent* event) {
    (void)machine;
    (void)event;
    return hfsm.res.ignore();
}

static void app_fsm_on_idle_entry(HfsmMachine* machine) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context != NULL) {
        context->current_state = APP_FSM_STATE_IDLE;
    }
}

static void app_fsm_on_manual_entry(HfsmMachine* machine) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context != NULL) {
        context->current_state = APP_FSM_STATE_MANUAL;
    }
}

static void app_fsm_on_fault_entry(HfsmMachine* machine) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context != NULL) {
        context->current_state = APP_FSM_STATE_FAULT;
    }
}

static void app_fsm_on_estop_entry(HfsmMachine* machine) {
    AppFsmContext* context = app_fsm_context_from_machine(machine);

    if(context != NULL) {
        context->current_state = APP_FSM_STATE_ESTOP;
    }
}
