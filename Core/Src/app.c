#include "app.h"
#include <stdio.h>
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

/* ==============================================================
 * 1. DMA 오디오 버퍼 및 설정
 * ============================================================== */
#define ADC_BUF_LEN    200      // DMA가 한 번에 모을 데이터 개수 (L: 100개, R: 100개)
uint16_t adc_buffer[ADC_BUF_LEN]; // 쏟아지는 마이크 값을 담을 빈 박스

#define DIFF_TH        220U     // 두 채널 레벨 차이 기준
#define ALPHA_DIV      4U       // IIR 필터 부드러움 정도
#define SWITCH_HOLDOFF 40U      // 방향 토글 방지(반사/노이즈) ms

// 인터럽트(백그라운드)와 메인 루프가 같이 쓰는 변수는 반드시 volatile을 붙여야 합니다.
static volatile uint32_t baseL = 0, baseR = 0;
static volatile uint32_t lvlL = 0,  lvlR = 0;
static volatile uint8_t is_calibrated = 0; // 영점 조절 완료 플래그

static char detectLR = '-';
static uint32_t last_switch_ms = 0;

/* ==============================================================
 * 2. 모터 제어용 변수
 * ============================================================== */
#define SERVO_LEFT_US   1200u
#define SERVO_RIGHT_US  1820u
#define SERVO_CENTER_US 1520u
#define SERVO_RUN_MS    200u
#define DIR_COOLDOWN_MS 400u

static uint32_t motor_lock_until_ms = 0;
static char last_dir = '-';
static uint8_t motor_running = 0;
static uint32_t motor_stop_ms = 0;
static char pending_dir = '-';

/* ==============================================================
 * 3. 유틸리티 함수
 * ============================================================== */
static inline uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* ==============================================================
 * 4. 초기화 함수 (main.c에서 1회 호출됨)
 * ============================================================== */
void app_init(void)
{
    detectLR = '-';
    lvlL = lvlR = 0;
    is_calibrated = 0; // 시작할 때 영점 조절 상태 초기화
    last_switch_ms = HAL_GetTick();

    // 모터 초기화
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CENTER_US);

    printf("Starting DMA Audio System...\r\n");

    // 🌟 핵심: 여기서 딱 한 번만 명령하면, 이후엔 하드웨어가 무한 반복해서 배열을 채웁니다.
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUF_LEN);
}

/* ==============================================================
 * 5. DMA 인터럽트 콜백 함수 (배열이 200개 꽉 찰 때마다 알아서 실행됨)
 * ============================================================== */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) {

        // ① 최초 1회 영점(Baseline) 캘리브레이션
        if (!is_calibrated) {
            uint32_t sumL = 0, sumR = 0;
            // 배열을 훑으며 평균을 구합니다 (짝수: L, 홀수: R)
            for(int i = 0; i < ADC_BUF_LEN; i += 2) {
                sumL += adc_buffer[i];
                sumR += adc_buffer[i+1];
            }
            baseL = sumL / (ADC_BUF_LEN / 2);
            baseR = sumR / (ADC_BUF_LEN / 2);
            is_calibrated = 1; // 캘리브레이션 끝!
            return;
        }

        // ② 배열 안에서 가장 큰 소리(Peak) 찾기
        uint32_t max_magL = 0;
        uint32_t max_magR = 0;

        for(int i = 0; i < ADC_BUF_LEN; i += 2) {
            uint32_t magL = u32_abs_diff(adc_buffer[i], baseL);
            uint32_t magR = u32_abs_diff(adc_buffer[i+1], baseR);

            if(magL > max_magL) max_magL = magL;
            if(magR > max_magR) max_magR = magR;
        }

        // ③ 찾아낸 최고 볼륨을 IIR 필터에 통과 (부드럽게 만들기)
        lvlL = lvlL + (uint32_t)(((int32_t)max_magL - (int32_t)lvlL) / (int32_t)ALPHA_DIV);
        lvlR = lvlR + (uint32_t)(((int32_t)max_magR - (int32_t)lvlR) / (int32_t)ALPHA_DIV);
    }
}

/* ==============================================================
 * 6. 메인 무한 루프
 * ============================================================== */
void app_loop(void)
{
    // 영점 조절이 안 끝났으면 모터 제어 대기
    if (!is_calibrated) return;

    // 🌟 핵심: 더 이상 여기서 ADC를 읽기 위해 기다리지 않습니다. (adc_readLR 삭제됨)
    // 백그라운드 인터럽트에서 계산해 준 lvlL, lvlR 값을 그냥 날름 가져다 씁니다.

    uint32_t diff = (lvlL > lvlR) ? (lvlL - lvlR) : (lvlR - lvlL);

    // 방향 판정
    if (diff >= DIFF_TH)
    {
        char newDir = (lvlL > lvlR) ? 'L' : 'R';
        uint32_t now = HAL_GetTick();

        if (newDir != detectLR && (now - last_switch_ms >= SWITCH_HOLDOFF)) {
            detectLR = newDir;
            last_switch_ms = now;
        }
    }

    // 디버그 출력
    static uint32_t last_dbg = 0;
    uint32_t nowm = HAL_GetTick();
    if (nowm - last_dbg >= 200) {
        printf("lvlL=%u lvlR=%u diff=%u detect=%c\r\n",
               (unsigned)lvlL, (unsigned)lvlR, (unsigned)diff, detectLR);
        last_dbg = nowm;
    }

    /* ==================================
       방향 변화 감지 및 모터 구동 (기존 로직 동일)
       ================================== */
    if (detectLR != last_dir) {
        last_dir = detectLR;

        if (detectLR == 'L' || detectLR == 'R') {
            if ((int32_t)(nowm - motor_lock_until_ms) < 0) {
                return;
            }

            if (motor_running) {
                pending_dir = detectLR;
            } else {
                pending_dir = '-';
                motor_running = 1;
                motor_stop_ms = nowm + SERVO_RUN_MS;

                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                                      (detectLR == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);

                motor_lock_until_ms = nowm + SERVO_RUN_MS + DIR_COOLDOWN_MS;
            }
        }
    }

    if (motor_running && (int32_t)(nowm - motor_stop_ms) >= 0) {
        motor_running = 0;
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

        if (pending_dir == 'L' || pending_dir == 'R') {
            char dir = pending_dir;
            pending_dir = '-';

            motor_running = 1;
            motor_stop_ms = nowm + 20 + SERVO_RUN_MS;

            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                                  (dir == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);
            motor_lock_until_ms = nowm + 20 + SERVO_RUN_MS + DIR_COOLDOWN_MS;
        }
    }
}
