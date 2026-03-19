# EdgeDevice STM32 (NUCLEO-F401RE)

STM32F401RE 기반 엣지 디바이스 펌웨어입니다.
현재 프로젝트는 아래 기능을 통합합니다.

- I2S 듀얼 마이크 기반 좌/우 방향 감지
- PAN/TILT 서보 제어 (AUTO/MANUAL)
- AHT10 온습도 측정 + 이동 평균 필터
- PCF8591 조도 측정
- UART1(RPi) 바이너리 프레임 + JSON 프로토콜
- FreeRTOS 기반 태스크 분리
- RPi 외부 전원 릴레이 감시(PC3/PC10) 코드 경로

---

## 1) 현재 주요 기능

- AUTO 모드
- 마이크 입력 신호를 처리해 방향(`L/R/-`)을 계산
- 계산된 방향에 따라 PAN 서보를 추적 제어

- MANUAL 모드
- UART 명령(`w/a/s/d`)으로 PAN/TILT 수동 제어
- 이동 명령 버스트는 coalescing(최신 1개) + 최소 적용 간격으로 안정화

- 센서
- AHT10: 2초 주기(최소 1초 강제), 변환 대기, BUSY 재시도, N=10 이동 평균
- PCF8591: CH0 조도값(0~255) 주기 갱신

- 통신
- UART1(RPi): `CMD(1) + LEN(4, BE) + PAYLOAD(JSON)`
- UART2(터미널): 디버그/모니터링 출력

---

## 2) 핀 맵 (현재 기준)

- 서보 PWM
- `PA6`: TIM3_CH1 (PAN)
- `PA7`: TIM3_CH2 (TILT)

- 마이크 I2S
- `PB12`: I2S2_WS
- `PB13`: I2S2_CK
- `PB15`: I2S2_SD

- I2C 센서
- `PB8`: I2C1_SCL
- `PB9`: I2C1_SDA

- UART
- `PA9/PA10`: USART1 (RPi)
- `PA2/PA3`: USART2 (터미널)

- 기타 GPIO
- `PB0/PB1/PB2`: IR LED 출력
- `PC3`: RPi 하트비트 입력 (`RPI_HB`)
- `PC10`: 릴레이 제어 출력 (`RELAY_EN`)

---

## 3) FreeRTOS 구조

- `UartRxTask` (AboveNormal)
- UART1 바이트 수신 큐 소비
- 프레임 조립 파서 입력(`app_on_uart1_byte`)

- `ControlTask` (Normal, 10ms)
- 프로토콜 프레임 처리
- 모드/수동 명령 처리
- AUTO/MANUAL 모터 제어 루프

- `SensorTask` (BelowNormal, 20ms)
- AHT10/PCF8591 주기 처리
- 상태 출력 주기 관리

- `defaultTask`
- 현재 RPi 외부 watchdog 루프를 수행하도록 구성

- 추가 생성 태스크
- `SystemMonitorTask` (코드에서 생성)
- watchdog 카운터 기반 상태 확인 + IWDG refresh/중단 판단

---

## 4) UART 프로토콜

- 공통 프레임
- `CMD(1byte) + LEN(4byte big-endian) + PAYLOAD(JSON UTF-8)`

- 상태 요청
- 요청: `CMD=0x05`, `LEN=0`
- 응답: `CMD=0x05`, JSON
- 예: `{"tmp":25.34,"hum":48.12,"dir":"L","tilt":91,"light":123}`

- 모터 명령
- 요청: `CMD=0x04`, JSON
- 허용 토큰: `w`, `a`, `s`, `d`, `auto`, `unauto`
- 응답: `CMD=0x04`, ACK JSON
- 예: `{"ok":1,"mode":"manual","cmd":"w"}`

---

## 5) Watchdog 상태 (중요)

- 외부 watchdog 코드 경로
- `PC3` 하트비트 감시, 타임아웃 시 `PC10` 릴레이 제어

- 내부 IWDG 경로
- 코드에는 `HAL_IWDG_MODULE_ENABLED` 가드 기반 로직이 포함됨
- 실제 동작하려면 `.ioc`에서 IWDG Enable 후 코드 재생성이 필요

---

## 6) 빌드/운용 주의

- CubeMX 재생성 시 USER CODE 블록 밖 수정은 유실될 수 있음
- UART 과부하 환경에서는 UART1 진단 카운터(`DR/O/F/N/P/IV/OV/R/RO/RF`)를 먼저 확인
- I2S 경로 사용 중 ADC 보조 경로는 조건부 컴파일로 유지될 수 있음
- 모터 전원은 외부 5V 전원 + GND 공통을 권장

---

## 7) 주요 파일

- `Core/Src/app.c`: 프로토콜/모드/제어 루프 통합
- `Core/Src/mic.c`: I2S 샘플 처리 및 방향 추정
- `Core/Src/motor_ctrl.c`: AUTO/MANUAL 서보 제어
- `Core/Src/aht10.c`: 온습도 상태머신 + 필터
- `Core/Src/pcf8591.c`: 조도 센서 처리
- `Core/Src/main.c`: HW 초기화, RTOS 태스크 생성, watchdog 루프
- `irrled.ioc`: 핀/클럭/RTOS 기본 설정
