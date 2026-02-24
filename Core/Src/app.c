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

// 인터럽트(백그라운드)와 메인 루프랑 같이 변수를 쓰기 때문에 volatile 적용
static volatile uint32_t baseL = 0, baseR = 0;
static volatile uint32_t lvlL = 0,  lvlR = 0;
static volatile uint8_t is_calibrated = 0; // 영점 조절 완료 플래그

static char detectLR = '-';
static uint32_t last_switch_ms = 0;





static uint32_t next_ms = 0;
static uint8_t state = 0;
static volatile uint32_t rawL = 0, rawR = 0;  // 마지막 원본 ADC 값 보관


/* mic sensor ------=======---=-=--=@#@!$#@$#$#$@#$-===--*/
#define CAL_N          300U     // baseline 캘리브레이션 샘플 수
#define DIFF_TH        220U     // 요구사항: 두 채널 레벨 차이 기준
#define SOUND_TH       70U     // 전체 레벨(소리 존재) 기준 (환경에 맞춰 조정)
#define ALPHA_DIV      4U       // IIR: /8이면 반응 빠름, /16이면 더 부드러움
#define SWITCH_HOLDOFF 40U     // 방향 토글 방지(반사/노이즈) ms






// PA4=Left, PA5=Right로 가정
static uint32_t baseL = 0, baseR = 0;
static uint32_t lvlL = 0,  lvlR = 0;       // 엔벨로프(레벨)
static char detectLR = '-';
static char lastPrintedLR = '\0';
static uint32_t last_switch_ms = 0;

static inline uint32_t u32_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}




/* motor control code -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=!@#%^$%$^%$^%$^^%$%$%$---begin*/
#define SERVO_LEFT_US   1200u
#define SERVO_RIGHT_US  1820u
#define SERVO_CENTER_US 1520u
#define SERVO_RUN_MS    200u
#define DIR_COOLDOWN_MS 400u   // 예: 0.4초 동안 다음 방향 명령 무시(원하는 값으로)

static uint32_t motor_lock_until_ms = 0;  // 이 시간 전엔 새 명령 금지



static uint8_t servo_enabled = 0;
static uint32_t servo_next_ms = 0;
static uint8_t servo_state = 0;
static char last_dir = '-';
static uint8_t motor_running = 0;
static uint32_t motor_stop_ms = 0;
static char pending_dir = '-';

/* motor control code -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=!@#%^$%$^%$^%$^^%$%$%$---end*/







// ADC1 스캔 2채널 읽기: Rank1->L(PA4), Rank2->R(PA5)
static int adc_readLR(uint32_t *outL, uint32_t *outR)
{
    if (HAL_ADC_Start(&hadc1) != HAL_OK) return 0;

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) { HAL_ADC_Stop(&hadc1); return 0; }
    uint32_t vL = HAL_ADC_GetValue(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK) { HAL_ADC_Stop(&hadc1); return 0; }
    uint32_t vR = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    *outL = vL;
    *outR = vR;
    return 1;
}

void app_init(void)
{
    detectLR = '-';
    lastPrintedLR = '\0';
    lvlL = lvlR = 0;
    last_switch_ms = HAL_GetTick();

    // baseline 캘리브레이션(부팅 직후 조용할 때)
    uint64_t sL = 0, sR = 0;
    uint32_t ok = 0;

    for (uint32_t i = 0; i < CAL_N; i++)
    {
        uint32_t aL, aR;
        if (adc_readLR(&aL, &aR)) {
            sL += aL; sR += aR;
            ok++;
        }
    }

    if (ok == 0) {
        baseL = baseR = 0;
    } else {
        baseL = (uint32_t)(sL / ok);
        baseR = (uint32_t)(sR / ok);
    }

    // 초기 레벨도 baseline 기반으로 한 번 세팅(선택)
    uint32_t aL, aR;
    if (adc_readLR(&aL, &aR)) {

        lvlL = u32_abs_diff(aL, baseL);
        lvlR = u32_abs_diff(aR, baseR);
    }

    // motor init-----------------------------------------------
    servo_enabled = 0;
    servo_state = 0;
    servo_next_ms = HAL_GetTick();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CENTER_US);




    printf("PCLK1=%lu TIM3CLK=%lu\r\n",
           HAL_RCC_GetPCLK1Freq(),
           HAL_RCC_GetPCLK1Freq()*2); // APB1 prescaler가 /2일 때 TIM클럭은 x2

}

void app_loop(void)
{








    uint32_t adcL, adcR;
    if (!adc_readLR(&adcL, &adcR)) return;
    rawL=adcL/2;
    rawR=adcR/2;
    // 1) 정류(부호/위상 문제 제거)
    uint32_t magL = u32_abs_diff(adcL, baseL);
    uint32_t magR = u32_abs_diff(adcR, baseR);

    // 2) 엔벨로프 추정(IIR 저역통과)
    // lvl = lvl + (mag - lvl)/ALPHA_DIV
    lvlL = lvlL + (uint32_t)(((int32_t)magL - (int32_t)lvlL) / (int32_t)ALPHA_DIV);
    lvlR = lvlR + (uint32_t)(((int32_t)magR - (int32_t)lvlR) / (int32_t)ALPHA_DIV);

    // 3) 방향 판정: 레벨 차이가 충분히 클 때만
    uint32_t diff  = (lvlL > lvlR) ? (lvlL - lvlR) : (lvlR - lvlL);
    uint32_t total = lvlL + lvlR;

//    if (total >= SOUND_TH && diff >= DIFF_TH)
    if (diff >= DIFF_TH)
    {
        char newDir = (lvlL > lvlR) ? 'L' : 'R';

        uint32_t now = HAL_GetTick();
        // 반대 방향으로 너무 자주 바뀌는 것 방지
        if (newDir != detectLR && (now - last_switch_ms >= SWITCH_HOLDOFF)) {
            detectLR = newDir;
            last_switch_ms = now;
        }
        // 같으면 유지(출력도 안 함)
    }

    // 4) 방향이 바뀔 때만 출력
//    if (detectLR != lastPrintedLR)
//    {
//        printf("DETECT=%c (lvlL=%u lvlR=%u diff=%u)\r\n",
//               detectLR, (unsigned)lvlL, (unsigned)lvlR, (unsigned)diff);
//        lastPrintedLR = detectLR;
//    }

    static uint32_t last_dbg = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_dbg >= 200) {
        printf("lvlL=%u lvlR=%u diff=%u total=%u detect=%c\r\n",
               (unsigned)lvlL, (unsigned)lvlR,
               (unsigned)diff, (unsigned)(lvlL+lvlR),
               detectLR);
        last_dbg = now;
    }



    /* 방향 변화 감지 */

    uint32_t nowm = HAL_GetTick();


    if (detectLR != last_dir) {
        last_dir = detectLR;

        if (detectLR == 'L' || detectLR == 'R') {

            // ===== 쿨다운: 락 시간 동안은 새 명령/예약을 무시 =====
            if ((int32_t)(nowm - motor_lock_until_ms) < 0) {
                // 아직 락 유지 중
                return; // app_loop 전체를 빠져나가도 되고, 여기만 스킵해도 됨
                // return이 싫으면: 그냥 모터 제어 블록만 건너뛰게 if로 감싸도 됨
            }

            if (motor_running) {
                pending_dir = detectLR;   // 동작 중이면 예약만
            } else {
                pending_dir = '-';
                motor_running = 1;
                motor_stop_ms = nowm + SERVO_RUN_MS;

                HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                                      (detectLR == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);

                // ===== 이번 동작 후 일정 시간까지 다음 명령 금지 =====
                motor_lock_until_ms = nowm + SERVO_RUN_MS + DIR_COOLDOWN_MS;
            }
        }
    }

    /* 동작 종료 처리 + 예약 실행 */
    if (motor_running && (int32_t)(nowm - motor_stop_ms) >= 0) {
        motor_running = 0;
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

        if (pending_dir == 'L' || pending_dir == 'R') {
            char dir = pending_dir;
            pending_dir = '-';

            motor_running = 1;
            motor_stop_ms = nowm + 20 + SERVO_RUN_MS;  // 20ms 안정 시간 포함

            HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                                  (dir == 'L') ? SERVO_LEFT_US : SERVO_RIGHT_US);
            motor_lock_until_ms = nowm + 20 + SERVO_RUN_MS + DIR_COOLDOWN_MS;
            // 디버그
            // printf("RUN PENDING: %c\r\n", dir);
        }
    }


}
