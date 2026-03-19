# EdgeDevice STM32 (NUCLEO-F401RE)

STM32F401RE 기반 엣지 디바이스 펌웨어입니다.  
현재 프로젝트는 **소리 방향 감지(I2S 마이크)**, **서보 팬/틸트 제어**, **온습도(AHT10)**, **조도(PCF8591)**, **RPi 연동(UART1 바이너리 프레임 + JSON)** 을 통합합니다.

---

## 1) 현재 기능 요약

- I2S 듀얼 채널 오디오 입력(INMP441 2개 가정)
- 좌/우 방향 감지 + AUTO 추적
- MANUAL 모드에서 팬/틸트 수동 이동
- AHT10 온습도 주기 측정 + 이동평균 필터
- PCF8591 조도(CH0) 주기 측정
- UART1(RPi) 프로토콜 요청/응답
- UART2(터미널) 단문 제어 + UART1 수신 프레임 미러 출력(옵션)
- FreeRTOS 기반 태스크 분리

---

## 2) 하드웨어/핀 매핑 (현재 코드 기준)

- `PA6` : `TIM3_CH1` (PAN 서보)
- `PA7` : `TIM3_CH2` (TILT 서보)
- `PB12` : `I2S2_WS`
- `PB13` : `I2S2_CK`
- `PB15` : `I2S2_SD`
- `PB8` : `I2C1_SCL` (AHT10 + PCF8591 공용)
- `PB9` : `I2C1_SDA` (AHT10 + PCF8591 공용)
- `PA9/PA10` : `USART1` (RPi 프로토콜)
- `PA2/PA3` : `USART2` (터미널)
- `PB0/PB1/PB2` : IR LED GPIO 출력

---

## 3) 소프트웨어 아키텍처

### 3.1 Task 구성

- `UartRxTask` (AboveNormal)
  - `uart_rx_queue`에서 UART1 바이트 수신
  - 파서 상태머신에 바이트 축적(`app_on_uart1_byte`)
- `ControlTask` (Normal, 10ms)
  - 완성 프레임 처리(`drain_protocol_queue`)
  - ACK/상태응답 송신
  - `control_queue` 처리
  - AUTO: `mic_process` + `motor_ctrl_process`
  - MANUAL: `motor_ctrl_manual_process`
- `SensorTask` (BelowNormal, 20ms)
  - `aht10_process`, `pcf8591_process`
  - 주기 디버그 출력(500ms)
- `defaultTask`
  - 현재 실질 동작 없음(유지)

### 3.2 Queue/Mutex

- `uart_rx_queue` : `uint8_t`, depth 256
- `control_queue` : `uint8_t`, depth 64
- `uart_tx_mutex` : UART1 패킷 송신 보호(`proto_uart1_send_packet`)

---

## 4) 마이크 처리 로직 (mic.c)

### 4.1 입력 및 전처리

- I2S DMA 수신 완료 콜백에서 데이터 처리
- 24-bit signed 샘플 언팩 후 16-bit 스케일로 축소
- 채널별 동적 DC 오프셋 추적:
  - `dc_offset += (sample - dc_offset) >> DC_TRACK_SHIFT`
- 노이즈 게이트:
  - `|sample| < NOISE_GATE_TH` 는 0 처리

### 4.2 특징량 추출

- 프레임 평균 진폭(`frame_magL/R`)
- 단기 신호창(`SIG_WIN`) 평균
- 장기 노이즈창(`NOISE_WIN`) 평균
- `SNR(Q8)` 계산

### 4.3 방향 판정

아래 게이트를 모두 통과하면 방향 갱신:

- `SOUND_TH` (절대 신호 세기)
- `SNR_TH_Q8` (신호 대 잡음비)
- `DIR_RATIO_TH_Q8` (좌/우 우세 비율)
- 추가 보호:
  - `noise_ready` 워밍업 완료
  - `SWITCH_HOLDOFF` 방향 전환 간격
  - `motor_lock_until_ms` 락 시간

---

## 5) 모터 제어 로직 (motor_ctrl.c)

### 5.1 AUTO 모드

- 감지 방향(`L/R`)에 따라 PAN 목표 PWM 이동
- `SERVO_RUN_MS` 동안 구동 후 정지
- `DIR_COOLDOWN_MS` 동안 재트리거 제한
- 진행 중 반대 방향은 `pending_dir`로 보류 처리

### 5.2 MANUAL 모드

- `W/A/S/D` 입력 시 step 단위 PWM 이동
- 이동 후 `MANUAL_PULSE_MS`만 신호 출력하고 PWM 정지
  - 목적: 무부하 상태에서 서보 발열/소음 감소
- 제어 명령 burst 보호
  - 이동 명령은 최신 1개로 합쳐(coalesce) 처리
  - 최소 적용 간격 `MANUAL_CMD_MIN_INTERVAL_MS`(현재 90ms) 적용
  - 목적: 명령이 몰릴 때 급격한 점프/튀는 동작 완화

### 5.3 서보 안전 범위(현재 값)

- PAN: `1210 ~ 1810`, CENTER `1510`
- TILT: `1210 ~ 1810`, CENTER `1510`
- 이유: 실제 기구 한계/부하 편차 고려해 끝단 충돌 위험 완화

---

## 6) 센서 로직

### 6.1 AHT10 (aht10.c)

- 기본 주기 2초 (`period_ms`), 최소 1초 강제
- 측정 명령 후 변환 대기 80ms
- BUSY 비트 확인 후 재시도
- 최근 `N=10` 이동평균으로 최종 온습도 산출

출력 스케일:

- `temperature_c_x100`
- `humidity_rh_x100`

### 6.2 PCF8591 (pcf8591.c)

- CH0 조도 8-bit 원시값(0~255) 주기 측정
- 제어 바이트 전송 후 2바이트 수신
- 첫 바이트(dummy) 폐기, 두 번째 바이트 사용

---

## 7) UART 프로토콜

### 7.1 프레임 형식 (공통)

- `CMD(1byte) + LEN(4byte, big-endian) + PAYLOAD(JSON UTF-8)`

### 7.2 UART1 (RPi) 명령

1. 상태 요청
- 요청: `CMD=0x05`, `LEN=0`
- 응답: `CMD=0x05`, JSON
- 응답 예:
  - `{"tmp":25.34,"hum":48.12,"dir":"L","tilt":91,"light":123}`

2. 모터 명령
- 요청: `CMD=0x04`, JSON 예:
  - `{"motor":"w"}`
  - `{"motor":"auto"}`
  - `{"motor":"unauto"}`
- 허용 토큰:
  - `w`, `a`, `s`, `d`, `auto`, `unauto`
- 응답: `CMD=0x04`, ACK JSON
  - `{"ok":1,"mode":"manual","cmd":"w"}`

토큰 매핑:

- `auto` -> 내부 명령 `'o'` (AUTO)
- `unauto` -> 내부 명령 `'f'` (MANUAL)

### 7.3 UART2 (터미널) 단문 명령

- `o/O`: AUTO 모드
- `f/F`: MANUAL 모드
- `w/a/s/d`: MANUAL 이동
  - AUTO 상태에서 들어와도 MANUAL로 전환 후 적용

### 7.4 UART 파서 안정화 포인트

- CMD 화이트리스트: `0x04/0x05` 외 바이트 폐기
- 프레임 타임아웃: `PROTO_RX_TIMEOUT_MS=100ms`
- 완료 프레임 링큐: `PROTO_FRAME_QUEUE_LEN=8`
- 상태 응답 레이트 제한: `STATUS_REPLY_MIN_INTERVAL_MS=100ms`
- 효과:
  - `IV/OV` 연쇄 증가 완화
  - 요청 폭주 시 응답 몰림 완화
  - 제어 루프 지연 완화

---

## 8) 디버그 출력

### 8.1 AUTO 출력

- `I2S_L/I2S_R`
- `FINAL_L/FINAL_R`
- `DIR`
- `PAN/TILT`
- `T/H/LIGHT`
- `U1[...]` UART1 진단 카운터

### 8.2 MANUAL 출력

- 모드, PAN/TILT 각도
- PAN/TILT PWM
- LIGHT
- `U1[...]` UART1 진단 카운터

### 8.3 UART1 수신 프레임 미러(UART2)

- 옵션 매크로:
  - `UART1_RX_MIRROR_TO_UART2=1`일 때 활성
- 출력 예:
  - `[U1->STM] CMD:0x04 LEN:13 PAYLOAD:{"motor":"w"}`
- 용도:
  - RPi에서 실제로 어떤 프레임이 STM에 도착했는지 현장에서 즉시 확인

---

## 9) 빌드/운용 주의사항

- CubeMX 코드 재생성 시 USER CODE 밖 수정 내용은 덮어써질 수 있음
- HAL 모듈 매크로(`HAL_I2S_MODULE_ENABLED` 등) 상태에 따라 경로가 달라짐
- I2S는 DMA + ISR 기반이므로 ISR 부하가 커지면 UART 수신 지연이 생길 수 있음
- 현재 프로젝트는 일부 예외(예: I2S 에러 콜백)에서 자동 복구 로직이 충분하지 않음

---

## 10) 주요 파일

- `Core/Src/app.c` : 모드/프로토콜/루프 통합 제어
- `Core/Src/mic.c` : I2S 샘플 처리 및 방향 판정
- `Core/Src/motor_ctrl.c` : AUTO/MANUAL 서보 제어
- `Core/Src/aht10.c` : 온습도 상태머신 + 필터
- `Core/Src/pcf8591.c` : 조도 센서 처리
- `Core/Src/main.c` : 하드웨어 초기화, RTOS Task 생성
- `irrled.ioc` : 핀/클럭/주변장치 설정 원본
