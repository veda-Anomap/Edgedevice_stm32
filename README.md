# EdgeDevice STM32 (NUCLEO-F401RE)

이 프로젝트는 STM32에서 다음 기능을 수행합니다.
- 듀얼 마이크(ADC + DMA) 기반 좌/우 방향 감지
- 팬/틸트 2축 서보 제어(TIM3 CH1/CH2 PWM)
- AHT10 온습도 센서(I2C1) 측정
- UART 명령으로 자동/수동 모드 전환

## 현재 기능

### 1) 마이크 방향 감지
- 2개 ADC 채널을 DMA 인터리브(`L,R,L,R...`)로 샘플링합니다.
- `mic.c` 처리 흐름:
  - 기준선 보정(`|raw - base|`)
  - 짧은 창 신호 추적
  - 긴 창 노이즈 추적
  - SNR 게이트 + 좌우 비율 게이트로 방향 확정
- 주요 튜닝 상수:
  - `SOUND_TH`
  - `SNR_TH_Q8`
  - `DIR_RATIO_TH_Q8`
  - `SIG_WIN`, `NOISE_WIN`

### 2) 팬/틸트 서보 제어
- 팬(Pan): `TIM3_CH1` (PA6)
- 틸트(Tilt): `TIM3_CH2` (PA7)
- 현재 PWM 설정:
  - Prescaler: `83`
  - Period: `19999`
  - 서보 50Hz 주기 기준

### 3) AHT10 온습도 센서
- I2C1 핀:
  - SCL: PB8
  - SDA: PB9
- 측정 주기: 기본 `2000 ms`
- 변환 대기: `80 ms`
- 필터: `N=10` 원형버퍼 이동평균(FIFO 동작)

### 4) UART 모드/수동 제어
- `app.c` 기준 UART2 명령:
  - `o` / `O`: 자동 모드(on)
  - `f` / `F`: 수동 모드(off)
  - 수동 모드 이동:
    - `W/w`: 틸트 위
    - `S/s`: 틸트 아래
    - `A/a`: 팬 왼쪽
    - `D/d`: 팬 오른쪽
- 자동 모드 진입 시 팬/틸트가 중앙으로 복귀합니다.

### 5) LED 모듈 분리
- LED 제어는 `ir_led.c/.h`로 분리되어 있습니다.
- 부팅 시 PB0/PB1/PB2를 ON으로 설정합니다.

## 런타임 동작 요약

- `app_init()`
  - motor/mic/AHT10 초기화
  - UART RX 인터럽트 시작(1바이트 명령 수신)
- `app_loop()`
  - AHT10 상태머신은 항상 수행
  - 자동 모드에서만:
    - `mic_process()`
    - `motor_ctrl_process()`
    - 200ms 주기 디버그 출력
- 수동 모드에서는 마이크 디버그 출력이 비활성화됩니다.

## 핀맵

- `PA6` -> `TIM3_CH1` (팬 서보)
- `PA7` -> `TIM3_CH2` (틸트 서보)
- `PB8` -> `I2C1_SCL` (AHT10)
- `PB9` -> `I2C1_SDA` (AHT10)
- `PA2` -> `USART2_TX` (터미널/디버그)
- `PA3` -> `USART2_RX` (명령 입력)
- `PA9` -> `USART1_TX` (예약/확장)
- `PA10` -> `USART1_RX` (예약/확장)
- `PB0/PB1/PB2` -> IR LED GPIO 출력

## 소스 구조

- `Core/Src/app.c`: 앱 오케스트레이션, 모드 처리, UART 콜백
- `Core/Src/mic.c`: ADC DMA 오디오 처리/방향 판정
- `Core/Src/motor_ctrl.c`: 팬/틸트 자동/수동 제어 로직
- `Core/Src/aht10.c`: AHT10 프로토콜/변환 대기/이동평균 필터
- `Core/Src/ir_led.c`: IR LED GPIO 제어

## 빌드 참고

- 프로젝트는 `irrled.ioc` 기반으로 생성됩니다.
- 새 파일 추가 후 컴파일 누락 시:
  1. Project Refresh
  2. Clean
  3. Build

## 다음 단계(예정)

- UART JSON 프레임 프로토콜(`0x04` / `0x05` + length + payload) 통합 예정
- 현재 활성 명령 체계는 1바이트 명령(`o/f/WASD`)입니다.
