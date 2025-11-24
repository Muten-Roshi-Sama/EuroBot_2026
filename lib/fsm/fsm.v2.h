#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "Demarage.h"
#include "EmergencyStop.h"

typedef enum {
    FSM2_INIT,
    FSM2_IDLE,
    FSM2_LOST,
    FSM2_UPDATE_ISR,
    FSM2_DO_ISR,
    FSM2_STOP
} fsm2_state_t;

typedef struct {
    fsm2_state_t state;
    uint32_t last_tick_ms;
    bool ok_event;
    bool cl_event;
} fsm2_context_t;

void fsm2_init(fsm2_context_t *ctx, uint8_t demarage_pin, uint8_t stop_pin);
void fsm2_step(fsm2_context_t *ctx, uint32_t now_ms);
void fsm2_trigger_ok(fsm2_context_t *ctx);
void fsm2_trigger_cl(fsm2_context_t *ctx);