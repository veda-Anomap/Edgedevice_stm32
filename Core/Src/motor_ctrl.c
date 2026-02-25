#include "motor_ctrl.h"

/* Auto tracking behavior */
#define SERVO_RUN_MS      200U
#define DIR_COOLDOWN_MS  1500U

static TIM_HandleTypeDef *s_htim = NULL;

/* Shared servo state */
static uint16_t current_pan_pwm = PAN_CENTER;
static uint16_t current_tilt_pwm = TILT_CENTER;

/* Auto state machine */
static uint32_t motor_lock_until_ms = 0U;
static char last_dir = '-';
static uint8_t motor_running = 0U;
static uint32_t motor_stop_ms = 0U;
static char pending_dir = '-';

static uint16_t clamp_u16(int32_t v, uint16_t min_v, uint16_t max_v)
{
    if (v < (int32_t)min_v) return min_v;
    if (v > (int32_t)max_v) return max_v;
    return (uint16_t)v;
}

static inline void write_pan(uint16_t pwm_us)
{
    if (s_htim == NULL) return;
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, pwm_us);
}

static inline void write_tilt(uint16_t pwm_us)
{
    if (s_htim == NULL) return;
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_2, pwm_us);
}

static inline void motor_start_dir(char dir)
{
    current_pan_pwm = (dir == 'L') ? PAN_LEFT : PAN_RIGHT;
    write_pan(current_pan_pwm);
}

void motor_ctrl_init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;

    last_dir = '-';
    pending_dir = '-';
    motor_running = 0U;
    motor_lock_until_ms = 0U;
    motor_stop_ms = 0U;

    current_pan_pwm = PAN_CENTER;
    current_tilt_pwm = TILT_CENTER;

    write_pan(current_pan_pwm);
    write_tilt(current_tilt_pwm);
}

uint32_t motor_ctrl_get_lock_until_ms(void)
{
    return motor_lock_until_ms;
}

void motor_ctrl_enter_manual(void)
{
    pending_dir = '-';
    motor_running = 0U;
    motor_lock_until_ms = 0U;
    write_pan(current_pan_pwm);
    write_tilt(current_tilt_pwm);
}

void motor_ctrl_enter_auto(void)
{
    /* Auto mode re-entry: reset servo pose to center */
    pending_dir = '-';
    motor_running = 0U;
    motor_lock_until_ms = 0U;
    last_dir = '-';

    current_pan_pwm = PAN_CENTER;
    current_tilt_pwm = TILT_CENTER;
    write_pan(current_pan_pwm);
    write_tilt(current_tilt_pwm);
}

void manual_move_pan(int step)
{
    const int32_t next = (int32_t)current_pan_pwm + (int32_t)step;
    current_pan_pwm = clamp_u16(next, PAN_LEFT, PAN_RIGHT);
    write_pan(current_pan_pwm);
}

void manual_move_tilt(int step)
{
    const int32_t next = (int32_t)current_tilt_pwm + (int32_t)step;
    current_tilt_pwm = clamp_u16(next, TILT_UP, TILT_DOWN);
    write_tilt(current_tilt_pwm);
}

uint16_t motor_ctrl_get_pan_pwm(void)
{
    return current_pan_pwm;
}

uint16_t motor_ctrl_get_tilt_pwm(void)
{
    return current_tilt_pwm;
}

void motor_ctrl_process(uint32_t now_ms, char detect_dir)
{
    if (detect_dir != last_dir) {
        last_dir = detect_dir;

        if (detect_dir == 'L' || detect_dir == 'R') {
            if ((int32_t)(now_ms - motor_lock_until_ms) < 0) {
                return;
            }

            if (motor_running) {
                pending_dir = detect_dir;
            } else {
                pending_dir = '-';
                motor_running = 1U;
                motor_stop_ms = now_ms + SERVO_RUN_MS;

                motor_start_dir(detect_dir);
                motor_lock_until_ms = now_ms + SERVO_RUN_MS + DIR_COOLDOWN_MS;
            }
        }
    }

    if (motor_running && (int32_t)(now_ms - motor_stop_ms) >= 0) {
        motor_running = 0U;
        if (s_htim != NULL) {
            HAL_TIM_PWM_Stop(s_htim, TIM_CHANNEL_1);
        }

        if (pending_dir == 'L' || pending_dir == 'R') {
            const char dir = pending_dir;
            pending_dir = '-';

            motor_running = 1U;
            motor_stop_ms = now_ms + 20U + SERVO_RUN_MS;

            motor_start_dir(dir);
            motor_lock_until_ms = now_ms + 20U + SERVO_RUN_MS + DIR_COOLDOWN_MS;
        }
    }
}
