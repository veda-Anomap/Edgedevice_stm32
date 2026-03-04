# Version History

프로젝트 변경 이력입니다.  
날짜 형식: `YYYY-MM-DD`

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

