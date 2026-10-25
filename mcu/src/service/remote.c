/**
 * @file remote.c
 * @brief FS-iA10B remote input service implementation
 */

#include "remote.h"
#include "delay.h"

#include <string.h>

#define REMOTE_TIMEOUT_MS 100u

static FsIa10b* s_receiver = NULL;
static RemoteState s_remote_state = { 0 };
static bool s_clear_reset_event_pending = false;
static bool s_clear_reset_edge_armed = false;
static bool s_clear_condition_active = false;

void remote_set_receiver(FsIa10b* receiver) {
    s_receiver = receiver;
}

void remote_init(void) {
    memset(&s_remote_state, 0, sizeof(s_remote_state));
    s_clear_reset_event_pending = false;
    s_clear_reset_edge_armed = false;
    s_clear_condition_active = false;
}

void remote_process(void) {
    FsIa10bData rc_data;
    bool clear_condition;

    if(s_receiver == NULL || fs_ia10b_maintain(s_receiver) != FS_IA10B_STATUS_OK ||
       fs_ia10b_get_data(s_receiver, &rc_data) != FS_IA10B_STATUS_OK ||
       !fs_ia10b_is_online(s_receiver, REMOTE_TIMEOUT_MS)) {
        memset(&s_remote_state, 0, sizeof(s_remote_state));
        s_clear_reset_event_pending = false;
        s_clear_reset_edge_armed = false;
        s_clear_condition_active = false;
        return;
    }

    s_remote_state.online = true;
    s_remote_state.rc_data = rc_data;
    s_remote_state.stamp_ms = rc_data.last_update_ms;
    s_remote_state.manual_request = (rc_data.channel[REMOTE_CH_SWD] == REMOTE_SW_LOW &&
                                     rc_data.channel[REMOTE_CH_SWA] == REMOTE_SW_HIGH);
    clear_condition = (rc_data.channel[REMOTE_CH_SWC] == REMOTE_SW_HIGH &&
                       rc_data.channel[REMOTE_CH_VRA] <= REMOTE_VR_LOW_THRESHOLD &&
                       rc_data.channel[REMOTE_CH_VRB] <= REMOTE_VR_LOW_THRESHOLD);
    if(!clear_condition) {
        s_clear_condition_active = false;
        s_clear_reset_edge_armed = true;
    }
    else {
        if(s_clear_reset_edge_armed && !s_clear_condition_active) {
            s_clear_reset_event_pending = true;
        }
        s_clear_condition_active = true;
    }
}

bool remote_get_state(RemoteState* out) {
    if(out == NULL) {
        return false;
    }
    *out = s_remote_state;
    return true;
}

bool remote_is_online(uint32_t timeout_ms) {
    return s_remote_state.online && s_receiver != NULL && fs_ia10b_is_online(s_receiver, timeout_ms) &&
           (delay_now_ms() - s_remote_state.stamp_ms) <= timeout_ms;
}

bool remote_is_manual_requested(void) {
    return remote_is_online(REMOTE_TIMEOUT_MS) && s_remote_state.manual_request;
}

bool remote_take_clear_reset_event(void) {
    if(!remote_is_online(REMOTE_TIMEOUT_MS) || !s_clear_reset_event_pending) {
        return false;
    }
    s_clear_reset_event_pending = false;
    return true;
}

const FsIa10bData* remote_get_raw_data(void) {
    return remote_is_online(REMOTE_TIMEOUT_MS) ? &s_remote_state.rc_data : NULL;
}
