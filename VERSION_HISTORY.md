# Version History

프로젝트 변경 이력입니다.  
날짜 형식: `YYYY-MM-DD`

---

## [v0.9.4] - 2026-03-05
### Dead Code 정리(main.c)
- 대상 파일: `Core/Src/main.c`
- 변경 내용:
  - `osKernelStart()` 이후의 불필요한 `while` 블록 내부 dead code 정리
  - 과거 `app_loop()` 호출 흔적 및 미사용 지역 변수 제거
  - 주석을 `RTOS: should not reach here`로 명확화
- 이유:
  - 스케줄러 시작 후 해당 구간이 실행되지 않아 혼동을 유발하므로 가독성/유지보수성 개선

### HAL Timebase를 TIM11로 분리
- 대상 파일:
  - `Core/Src/stm32f4xx_hal_timebase_tim.c` (신규)
  - `Core/Src/stm32f4xx_it.c`
  - `Core/Inc/stm32f4xx_it.h`
  - `Core/Src/stm32f4xx_hal_msp.c`
  - `irrled.ioc`
- 변경 내용(코드):
  - 신규 `HAL_InitTick()` 구현:
    - `TIM11`을 1ms 주기로 설정/시작
    - `HAL_SuspendTick()`, `HAL_ResumeTick()` 구현
    - `HAL_TIM_PeriodElapsedCallback()`에서 `TIM11` update 이벤트 시 `HAL_IncTick()` 호출
  - IRQ 연결:
    - `TIM1_TRG_COM_TIM11_IRQHandler()` 추가
    - 핸들러 내부 `HAL_TIM_IRQHandler(&htim11)` 호출
  - `SysTick_Handler()`:
    - `HAL_IncTick()` 제거
    - FreeRTOS tick(`xPortSysTickHandler`) 전용으로 사용
  - MSP 설정:
    - `TIM11` clock enable/disable 추가
    - `TIM1_TRG_COM_TIM11_IRQn` priority/NVIC enable/disable 추가
- 변경 내용(ioc):
  - `TIM11` IP 및 가상 핀(`VP_TIM11_VS_ClockSourceINT`) 추가
  - `TIM11` NVIC 항목 추가
  - Timebase 분리 구성 반영
- 이유:
  - FreeRTOS SysTick과 HAL tick을 분리해 시간기반 충돌 가능성을 낮추고 실무 안정성 향상

---

## [v0.9.3] - 2026-03-05
### ControlTask / SensorTask 역할 분리 완료
- 대상 파일: `Core/Inc/app.h`, `Core/Src/app.c`, `Core/Src/main.c`
- 변경 목적:
  - 제어 경로와 센서 경로를 분리해서, 센서 처리 지연이 모터 제어 주기를 막지 않도록 개선
  - RTOS 역할 분리 원칙에 맞게 Task 책임을 명확화

### 1) app 계층 함수 분리
- `Core/Inc/app.h`
  - 추가 API:
    - `void app_control_loop(void);`
    - `void app_sensor_loop(void);`
- `Core/Src/app.c`
  - `app_control_loop()` 추가:
    - `drain_control_queue()`로 제어 명령 먼저 반영
    - AUTO 모드에서만 `mic_process()` + `motor_ctrl_process()` 실행
    - 모드/마이크/모터 제어 전담
  - `app_sensor_loop()` 추가:
    - `aht10_process(now)` 실행
    - 주기 로그(ADC/방향/온습도/PAN/TILT) 출력
    - 센서 갱신/상태 표시 전담
  - `app_loop()`는 하위호환 wrapper로 유지:
    - 내부에서 `app_sensor_loop()` + `app_control_loop()` 순서 호출

### 2) 제어 명령 반영 지점 단일화(ControlTask 전담)
- `Core/Src/app.c`
  - 신규 함수:
    - `push_control_command(uint8_t cmd)`:
      - `control_queue`에 제어 명령 push
    - `drain_control_queue(void)`:
      - `control_queue`를 non-blocking으로 비우며 명령 적용
    - `apply_control_command(uint8_t cmd)`:
      - 기존 `handle_uart_command` 역할을 대체
      - `o/f/w/a/s/d` 명령 실제 반영
  - 변경점:
    - UART2 RX ISR: 직접 제어 대신 `push_control_command()`만 수행
    - UART1 `0x04` 프레임 파싱 후: 직접 제어 대신 `push_control_command()` 수행
    - 즉, 모터/모드 상태 변경은 ControlTask에서만 발생

### 3) Task 주기 조정
- `Core/Src/main.c`
  - `StartControlTask()`:
    - `app_control_loop();`
    - `osDelay(10);`  (10ms 제어 주기)
  - `StartSensorTask()`:
    - `app_sensor_loop();`
    - `osDelay(20);`  (20ms 센서/상태 주기)

### 4) 최종 실행 흐름
- UART ISR:
  - 큐 적재만 수행하고 즉시 return
- UartRxTask:
  - `uart_rx_queue`에서 바이트 수신 -> 프레임 조립/파싱
  - `0x04` 명령은 `control_queue`로 전달
  - `0x05`는 상태 응답 처리
- ControlTask(10ms):
  - `control_queue` 명령 반영
  - AUTO 시 마이크 방향 판단 + 모터 제어
- SensorTask(20ms):
  - AHT10 상태머신/주기 갱신 + 로그 출력

---

## [v0.9.2] - 2026-03-05
### UartRxTask를 실제 UART1 프레임 파서로 활성화
- 대상 파일: `Core/Inc/app.h`, `Core/Src/app.c`, `Core/Src/main.c`
- 변경 목적:
  - UART 프로토콜 파싱을 `app_loop` 경로에서 분리하고 `UartRxTask` 전담으로 전환
  - RTOS 구조에 맞는 역할 분리(큐 소비/파싱/프레임 처리)
- 변경 상세:
  - `Core/Inc/app.h`
    - `void app_on_uart1_byte(uint8_t b);` 공개 API 추가
  - `Core/Src/app.c`
    - `app_on_uart1_byte()` 추가
      - 입력 바이트를 `proto_rx_feed_byte()` 상태머신에 투입
      - 완성 프레임을 `proto_pop_frame()`로 꺼내 `process_protocol_frame()` 처리
    - `app_loop()`에서 UART1 큐 드레인/프레임 처리 코드 제거
      - UART1 파싱 책임을 Task로 완전히 이동
  - `Core/Src/main.c`
    - `StartUartRxTask()`를 실사용 로직으로 변경
      - `osMessageQueueGet(uart_rx_queueHandle, ..., osWaitForever)`로 바이트 대기
      - 수신 바이트를 `app_on_uart1_byte(rx_byte)`로 전달
- 최종 동작 흐름:
  - ISR(UART1): 큐에 1바이트 적재 후 즉시 리턴
  - UartRxTask: 큐에서 바이트 수신 -> 프레임 조립 -> `0x04/0x05` 처리
  - ControlTask: `app_loop()`로 센서/모터 주기 제어 유지

---

## [v0.9.1] - 2026-03-05
### UART1 ISR 최소화(Queue 기반) 적용
- 대상 파일: `Core/Src/app.c`
- 변경 목적:
  - UART RX 인터럽트에서 파싱 부담을 제거하여 ISR 실행 시간을 최소화
  - 프레임 드롭/지연 위험 완화
- 변경 상세:
  - `HAL_UART_RxCpltCallback()`의 `USART1` 경로에서
    - 기존: `proto_rx_feed_byte(rx_data_rpi)` 직접 호출
    - 변경: `osMessageQueuePut(uart_rx_queueHandle, &rx_data_rpi, 0, 0)`로 1바이트 큐 적재 후 즉시 리턴
  - ISR 밖 파싱 함수 추가:
    - `proto_drain_uart_rx_queue()`
    - `osMessageQueueGet(..., timeout=0)`로 큐를 비우며 `proto_rx_feed_byte()` 호출
  - `app_loop()` 시작부에 `proto_drain_uart_rx_queue()` 호출 추가
    - ISR에서 적재된 UART1 바이트를 Task 컨텍스트에서 프레임 조립하도록 변경
- 추가 선언:
  - `#include "cmsis_os2.h"`
  - `extern osMessageQueueId_t uart_rx_queueHandle;`
- 참고:
  - 큐가 아직 생성되지 않은 초기 시점에는 UART1 바이트를 무시하도록 가드 처리(`uart_rx_queueHandle != NULL`)

---

## [v0.9.0] - 2026-03-04
### UART1(RPi) 바이너리 프레임 + JSON 파싱/응답 추가
- 대상 파일: `Core/Src/app.c`
- 추가 매크로/상수:
  - `PROTO_MAX_PAYLOAD` (최대 payload 192B)
- 추가 상태머신:
  - `proto_rx_state_t`
  - 상태: `PROTO_RX_WAIT_CMD -> PROTO_RX_WAIT_LEN_0..3 -> PROTO_RX_WAIT_PAYLOAD`
- 추가 버퍼/상태 변수:
  - `s_proto_state`, `s_proto_cmd`, `s_proto_len`, `s_proto_idx`
  - `s_proto_rx_buf[]`
  - `s_frame_ready`, `s_frame_cmd`, `s_frame_len`, `s_frame_payload[]`
- 추가 함수:
  - `proto_rx_feed_byte(uint8_t b)`
    - UART1에서 1바이트씩 입력받아 프레임 조립
    - 길이 필드 4바이트는 big-endian 해석
    - 길이 초과 시 reset
  - `proto_publish_frame(...)`
    - ISR 파싱 결과를 단일 슬롯 프레임 버퍼에 게시
  - `proto_pop_frame(...)`
    - 메인 로직에서 게시된 프레임 꺼냄(IRQ 보호)
  - `process_protocol_frame(...)`
    - `0x05`(상태요청) / `0x04`(모터명령) 분기 처리
  - `parse_motor_command(...)`
    - JSON 문자열에서 `"motor"` 키를 탐색
    - 값 토큰을 추출해 내부 단문 명령(`o/f/w/a/s/d`)으로 매핑
    - 매핑 허용값: `auto/manual/on/off/o/f/w/a/s/d`
  - `proto_uart1_send_packet(...)`
    - `CMD(1) + LEN(4) + JSON` 형태로 UART1 송신
  - `proto_send_status_packet()`
    - 상태 응답 JSON 생성:
    - `{"tmp":..,"hum":..,"dir":"..","tilt":..}`
  - `proto_send_motor_ack(...)`
    - 모터 명령 ACK JSON 생성:
    - `{"ok":1/0,"mode":"auto/manual","cmd":"..."}`
- UART 콜백 변경:
  - `HAL_UART_RxCpltCallback`에서
    - `USART2`: 기존 1바이트 수동 명령 처리 유지
    - `USART1`: `proto_rx_feed_byte()` 호출 후 인터럽트 재활성화
- 초기화 변경:
  - `app_init()`에서 `HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1)` 추가

### AUTO/MANUAL 출력 정보 확장
- 대상 파일: `Core/Src/app.c`
- 추가 함수:
  - `pwm_to_deg(uint16_t pwm, uint16_t min_pwm, uint16_t max_pwm)`
    - PWM 범위를 0~180도로 선형 변환
- 로그 변경:
  - AUTO 로그에 `PAN/TILT` 각도 추가
  - MANUAL 로그 추가:
    - `MODE:MANUAL | PAN:xdeg TILT:ydeg | PAN_PWM:... TILT_PWM:...`

### FreeRTOS 실행 경로 보정
- 대상 파일: `Core/Src/main.c`
- `StartControlTask()`에 `app_loop()` 호출 추가
  - `app_loop(); osDelay(5);`
- 의도:
  - `osKernelStart()` 이후 실질 동작이 Task 컨텍스트에서 수행되도록 보정

---

## [v0.8.0] - 2026-03-04
### RTOS 친화 지연 처리로 전환
- 대상 파일: `Core/Src/motor.c`, `Core/Src/servo_calc.c`
- 변경 이유:
  - RTOS 환경에서 `HAL_Delay` 블로킹 영향을 줄이기 위함
- 변경 내용:
  - `cmsis_os2.h` 포함
  - `delay_ms(uint32_t ms)` 헬퍼 추가
    - 커널 실행 중: `osDelay(ms)`
    - 커널 시작 전: `HAL_Delay(ms)`
  - 기존 `HAL_Delay(...)` 호출을 `delay_ms(...)`로 치환

---

## [v0.7.0] - 2026-03-03
### 자동/수동 모드 제어 체계 업데이트
- 대상 파일: `Core/Src/app.c`, `Core/Src/motor_ctrl.c`, `Core/Inc/motor_ctrl.h`
- 모드 명령:
  - `o/O` -> AUTO
  - `f/F` -> MANUAL
- 수동 명령:
  - `W/S`(틸트), `A/D`(팬)
- 함수 추가/사용:
  - `motor_ctrl_enter_auto()`
  - `motor_ctrl_enter_manual()`
  - `manual_move_pan(int step)`
  - `manual_move_tilt(int step)`
- AUTO 재진입 시 팬/틸트 센터 복귀 로직 추가

### 2축 팬틸트 제어 확정
- 대상 파일: `Core/Src/motor_ctrl.c`
- 채널:
  - PAN -> `TIM3_CH1`
  - TILT -> `TIM3_CH2`
- 상태 변수:
  - `current_pan_pwm`, `current_tilt_pwm`
- 방어 로직:
  - 한계 PWM clamp 처리

---

## [v0.6.0] - 2026-03-03
### 마이크 방향 판정 게이트 단순화
- 대상 파일: `Core/Src/mic.c`
- 의사결정 게이트를 핵심 3개로 단순화:
  - `SOUND_TH`
  - `SNR_TH_Q8`
  - `DIR_RATIO_TH_Q8`
- 제거된 방향 확정 조건:
  - `DIFF_TH`, `SNR_DIFF_TH_Q8`, `PEAK_DIFF_TH` (의사결정 경로에서 제외)
- 목적:
  - 튜닝 포인트 축소
  - 오탐/미탐 트레이드오프를 현장에서 빠르게 맞추기 쉽게 개선

---

## [v0.5.0] - 2026-03-03
### AHT10 필터링 강화 (N=10 FIFO 이동평균)
- 대상 파일: `Core/Src/aht10.c`
- 추가 상수/변수:
  - `AHT10_FILTER_WIN = 10`
  - `s_temp_hist[]`, `s_humi_hist[]`
  - `s_hist_idx`, `s_hist_count`, `s_temp_sum`, `s_humi_sum`
- 추가 함수:
  - `aht10_filter_push(...)`
- 동작:
  - 새 샘플 입력 시 가장 오래된 샘플 제거 후 합계 갱신(O(1))
  - 단일 샘플 출력 대신 최근 N개 평균 출력

---

## [v0.4.0] - 2026-03-03
### AHT10 모듈 분리 및 상태머신 적용
- 대상 파일: `Core/Src/aht10.c`, `Core/Inc/aht10.h`
- 구성:
  - `aht10_init()`, `aht10_process()`, `aht10_get_data()`
- 핵심 로직:
  - 주기 측정(기본 2000ms)
  - 변환 대기(약 80ms)
  - BUSY 비트 확인 후 재시도
- 목적:
  - 센서 자가발열 완화
  - 측정 완료 전 읽기 오류 방지

---

## [v0.3.0] - 2026-03-03
### 모듈 분리 리팩토링
- `mic`, `motor_ctrl`, `aht10`, `ir_led` 모듈 분리
- `app.c`를 연계 오케스트레이션 계층으로 정리

### IR LED 코드 분리
- 대상 파일: `Core/Src/ir_led.c`, `Core/Inc/ir_led.h`
- PB0/PB1/PB2 제어를 별도 모듈로 분리

---

## [v0.2.0] - 2026-03-03
### 수동 조작 기능 도입
- UART 단문 명령 기반 수동 조작 추가
- 팬/틸트 step 제어 및 모드 전환 동작 정리

---

## [v0.1.0] - 2026-03-03
### 초기 버전
- 듀얼 마이크 방향 감지 + 서보 구동 기본 동작
- DMA 기반 ADC 샘플 처리 경로 구성
