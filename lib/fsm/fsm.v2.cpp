#include <Arduino.h>
#include "fsm.v2.h"

void fsm2_trigger_ok(fsm2_context_t *ctx) { ctx->ok_event = true; }
void fsm2_trigger_cl(fsm2_context_t *ctx) { ctx->cl_event = true; }

void fsm2_init(fsm2_context_t *ctx, uint8_t demarage_pin, uint8_t stop_pin) {
    ctx->state = FSM2_INIT;
    ctx->last_tick_ms = 0;
    ctx->ok_event = false;
    ctx->cl_event = false;
    demarage_init(demarage_pin);
    emergency_stop_init(stop_pin);
}

void fsm2_step(fsm2_context_t *ctx, uint32_t now_ms) {
    demarage_update();
    emergency_stop_update();
    if (emergency_button_pressed()) {
        ctx->state = FSM2_STOP;
        return;
    }
    switch (ctx->state) {
        case FSM2_INIT:
            if (demarage_is_ready()) {
                ctx->state = FSM2_IDLE;
            }
            break;
        case FSM2_IDLE:
            ctx->state = FSM2_TASK;
            ctx->last_tick_ms = now_ms;
            break;
        case FSM2_TASK:
            if (ctx->ok_event) {
                ctx->state = FSM2_IDLE;
                ctx->ok_event = false;
            } else if (ctx->cl_event) {
                ctx->state = FSM2_UPDATE_ISR;
                ctx->cl_event = false;
                ctx->last_tick_ms = now_ms;
            } else if (now_ms - ctx->last_tick_ms > 100) {
                ctx->state = FSM2_STOP;
            }
            break;
        case FSM2_UPDATE_ISR:
            if (now_ms - ctx->last_tick_ms > 10) {
                ctx->state = FSM2_DO_ISR;
                ctx->last_tick_ms = now_ms;
            }
            break;
        case FSM2_DO_ISR:
            if (now_ms - ctx->last_tick_ms > 10) {
                ctx->state = FSM2_TASK;
                ctx->last_tick_ms = now_ms;
            }
            break;
        case FSM2_STOP:
            break;
    }
}