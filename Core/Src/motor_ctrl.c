#include "motor_ctrl.h"

/* 서보 구동 타이밍 */
#define SERVO_LEFT_US    1200U /* 좌회전 명령 PWM 펄스폭 */
#define SERVO_RIGHT_US   1820U /* 우회전 명령 PWM 펄스폭 */
#define SERVO_CENTER_US  1520U /* 중앙 PWM 펄스폭 */
#define SERVO_RUN_MS      200U /* 명령당 서보 구동 시간(ms) */
#define DIR_COOLDOWN_MS  1500U /* 명령 후 재트리거 방지 락(ms) */

static TIM_HandleTypeDef *s_htim = NULL;

/* 방향/서보 상태머신 */
static uint32_t motor_lock_until_ms = 0U; /* 이 시각 전에는 신규 명령 무시 */
static char last_dir = '-';               /* 이미 소비된 마지막 방향 */
static uint8_t motor_running = 0U;        /* 서보 명령 실행 중 여부 */
static uint32_t motor_stop_ms = 0U;       /* 현재 명령 종료 시각 */
static char pending_dir = '-';            /* 서보 바쁠 때 대기시킬 방향 */

static inline void motor_start_dir(char dir)
{
    if (s_htim == NULL) return;
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1,
                          (dir == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);
}

void motor_ctrl_init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;

    last_dir = '-';
    pending_dir = '-';
    motor_running = 0U;
    motor_lock_until_ms = 0U;
    motor_stop_ms = 0U;

    if (s_htim != NULL) {
        __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, SERVO_CENTER_US);
    }
}

uint32_t motor_ctrl_get_lock_until_ms(void)
{
    return motor_lock_until_ms;
}

void motor_ctrl_process(uint32_t now_ms, char detect_dir)
{
    /* 방향 변화 감지 시 서보 명령 스케줄링 */
    if (detect_dir != last_dir) {
        last_dir = detect_dir;

        if (detect_dir == 'L' || detect_dir == 'R') {
            if ((int32_t)(now_ms - motor_lock_until_ms) < 0) {
                return;
            }

            if (motor_running) {
                /* 서보가 바쁘면 최신 방향만 대기열로 유지 */
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

    /* 현재 명령 종료 후 필요 시 pending 명령 1회 실행 */
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
