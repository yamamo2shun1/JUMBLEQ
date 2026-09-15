/*
 * ui_uf2_control.c
 *
 * UF2 bootloader transition state machine. Owns the arm/hold/cancel/notice
 * state and the OLED-facing progress getters.
 */

#include "ui_control.h"
#include "ui_uf2_control_internal.h"

#include <string.h>

static const uint32_t UF2_ARM_WINDOW_MS      = 10000U;
static const uint32_t UF2_SWITCH_DEBOUNCE_MS = 30U;
static const uint32_t UF2_SWITCH_HOLD_MS     = 2000U;
static const uint32_t UF2_NOTICE_MS          = 2000U;

typedef struct
{
    volatile UI_Uf2TransitionState_t state;
    volatile uint32_t arm_started_ms;
    volatile bool displays_cleared;
    uint32_t raw_changed_ms;
    uint32_t hold_started_ms;
    uint32_t notice_started_ms;
    bool last_raw_pressed;
    bool stable_pressed;
    bool release_observed;
} uf2_bootloader_control_t;

static uf2_bootloader_control_t s_uf2 = {
    .state = UI_UF2_TRANSITION_IDLE,
};

UI_Uf2TransitionState_t ui_control_get_uf2_transition_state(void)
{
    const UI_Uf2TransitionState_t state = s_uf2.state;
    __DMB();
    return state;
}

uint8_t ui_control_get_uf2_seconds_remaining(void)
{
    const UI_Uf2TransitionState_t state = s_uf2.state;
    __DMB();
    if ((state != UI_UF2_TRANSITION_WAIT_RELEASE) &&
        (state != UI_UF2_TRANSITION_WAIT_HOLD) &&
        (state != UI_UF2_TRANSITION_HOLDING))
    {
        return 0U;
    }

    const uint32_t elapsed_ms = HAL_GetTick() - s_uf2.arm_started_ms;
    if (elapsed_ms >= UF2_ARM_WINDOW_MS)
    {
        return 0U;
    }

    const uint32_t remaining_ms = UF2_ARM_WINDOW_MS - elapsed_ms;
    return (uint8_t) ((remaining_ms + 999U) / 1000U);
}

void ui_control_notify_uf2_displays_cleared(void)
{
    if (s_uf2.state != UI_UF2_TRANSITION_CLEARING_DISPLAYS)
    {
        return;
    }

    __DMB();
    s_uf2.displays_cleared = true;
}

static bool uf2_transition_is_armed(UI_Uf2TransitionState_t state)
{
    return (state == UI_UF2_TRANSITION_WAIT_RELEASE) ||
           (state == UI_UF2_TRANSITION_WAIT_HOLD) ||
           (state == UI_UF2_TRANSITION_HOLDING) ||
           (state == UI_UF2_TRANSITION_CLEARING_DISPLAYS);
}

static bool uf2_confirmation_switch_pressed(void)
{
    // The front-panel control documented as SW3 is named SW2 in the MCU GPIO symbols.
    return (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET);
}

void ui_uf2_control_arm(uint8_t program_change)
{
    if (uf2_transition_is_armed(s_uf2.state))
    {
        SEGGER_RTT_printf(0, "UF2 bootloader request already armed\r\n");
        return;
    }

    const uint32_t now = HAL_GetTick();
    const bool pressed = uf2_confirmation_switch_pressed();

    s_uf2.arm_started_ms  = now;
    s_uf2.displays_cleared = false;
    s_uf2.raw_changed_ms  = now;
    s_uf2.hold_started_ms = 0U;
    s_uf2.notice_started_ms = 0U;
    s_uf2.last_raw_pressed = pressed;
    s_uf2.stable_pressed   = pressed;
    s_uf2.release_observed = false;
    __DMB();
    s_uf2.state            = UI_UF2_TRANSITION_WAIT_RELEASE;

    SEGGER_RTT_printf(0, "UF2 bootloader request armed for %lu ms (PC%u Ch15)\r\n",
                      (unsigned long) UF2_ARM_WINDOW_MS,
                      (unsigned) program_change);
}

static void set_uf2_transition_notice(UI_Uf2TransitionState_t state, const char* reason)
{
    s_uf2.hold_started_ms  = 0U;
    s_uf2.displays_cleared = false;
    s_uf2.notice_started_ms = HAL_GetTick();
    __DMB();
    s_uf2.state            = state;

    SEGGER_RTT_printf(0, "UF2 bootloader request %s\r\n", reason);
}

void ui_uf2_control_cancel(const char* reason)
{
    if (!uf2_transition_is_armed(s_uf2.state))
    {
        return;
    }

    set_uf2_transition_notice(UI_UF2_TRANSITION_CANCELLED, reason);
}

void ui_uf2_control_service(void)
{
    const uint32_t now = HAL_GetTick();
    UI_Uf2TransitionState_t state = s_uf2.state;

    if ((state == UI_UF2_TRANSITION_CANCELLED) || (state == UI_UF2_TRANSITION_TIMED_OUT))
    {
        if ((now - s_uf2.notice_started_ms) >= UF2_NOTICE_MS)
        {
            s_uf2.state = UI_UF2_TRANSITION_IDLE;
            __DMB();
        }
        return;
    }

    if (!uf2_transition_is_armed(state))
    {
        return;
    }

    if (!tud_mounted())
    {
        ui_uf2_control_cancel("cancelled because USB was disconnected");
        return;
    }

    if ((now - s_uf2.arm_started_ms) >= UF2_ARM_WINDOW_MS)
    {
        set_uf2_transition_notice(UI_UF2_TRANSITION_TIMED_OUT, "timed out");
        return;
    }

    const bool raw_pressed = uf2_confirmation_switch_pressed();
    if (raw_pressed != s_uf2.last_raw_pressed)
    {
        s_uf2.last_raw_pressed = raw_pressed;
        s_uf2.raw_changed_ms   = now;
    }

    if ((raw_pressed != s_uf2.stable_pressed) &&
        ((now - s_uf2.raw_changed_ms) >= UF2_SWITCH_DEBOUNCE_MS))
    {
        s_uf2.stable_pressed = raw_pressed;

        if (!raw_pressed)
        {
            s_uf2.release_observed = true;
            s_uf2.hold_started_ms  = 0U;
            s_uf2.displays_cleared = false;
            __DMB();
            s_uf2.state            = UI_UF2_TRANSITION_WAIT_HOLD;
            SEGGER_RTT_printf(0, "UF2 confirmation switch released; waiting for hold\r\n");
        }
        else if (s_uf2.release_observed)
        {
            s_uf2.hold_started_ms = now;
            __DMB();
            s_uf2.state           = UI_UF2_TRANSITION_HOLDING;
            SEGGER_RTT_printf(0, "UF2 confirmation switch hold started\r\n");
        }
    }

    state = s_uf2.state;
    if ((state == UI_UF2_TRANSITION_WAIT_RELEASE) &&
        !s_uf2.stable_pressed &&
        ((now - s_uf2.raw_changed_ms) >= UF2_SWITCH_DEBOUNCE_MS))
    {
        s_uf2.release_observed = true;
        __DMB();
        s_uf2.state            = UI_UF2_TRANSITION_WAIT_HOLD;
        SEGGER_RTT_printf(0, "UF2 confirmation switch release observed\r\n");
        return;
    }

    if ((state == UI_UF2_TRANSITION_HOLDING) &&
        s_uf2.stable_pressed &&
        ((now - s_uf2.hold_started_ms) >= UF2_SWITCH_HOLD_MS))
    {
        s_uf2.displays_cleared = false;
        __DMB();
        s_uf2.state = UI_UF2_TRANSITION_CLEARING_DISPLAYS;
        SEGGER_RTT_printf(0, "UF2 confirmation accepted; clearing displays before reset\r\n");
        return;
    }

    if ((state == UI_UF2_TRANSITION_CLEARING_DISPLAYS) &&
        s_uf2.displays_cleared &&
        raw_pressed &&
        s_uf2.stable_pressed)
    {
        __DMB();
        __DSB();
        NVIC_SystemReset();
        while (1)
        {
        }
    }
}

void ui_uf2_control_reset(void)
{
    memset(&s_uf2, 0, sizeof(s_uf2));
    s_uf2.state = UI_UF2_TRANSITION_IDLE;
}

// OLED表示用: stateと残り秒数を同じcaptureで取得する。scheduler停止区間専用。
void ui_uf2_control_capture_display_state(UI_Uf2DisplayState_t* state)
{
    if (state == NULL)
    {
        return;
    }

    const UI_Uf2TransitionState_t uf2_state = s_uf2.state;
    __DMB();

    state->state = uf2_state;
    state->seconds_remaining = 0U;

    if ((uf2_state != UI_UF2_TRANSITION_WAIT_RELEASE) &&
        (uf2_state != UI_UF2_TRANSITION_WAIT_HOLD) &&
        (uf2_state != UI_UF2_TRANSITION_HOLDING))
    {
        return;
    }

    const uint32_t elapsed_ms = HAL_GetTick() - s_uf2.arm_started_ms;
    if (elapsed_ms >= UF2_ARM_WINDOW_MS)
    {
        return;
    }

    const uint32_t remaining_ms = UF2_ARM_WINDOW_MS - elapsed_ms;
    state->seconds_remaining = (uint8_t) ((remaining_ms + 999U) / 1000U);
}
