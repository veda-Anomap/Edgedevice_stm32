# EdgeDevice STM32 (NUCLEO-F401RE)

STM32F401RE 기반 엣지 디바이스 프로젝트입니다.

현재 구현 범위:
- 듀얼 마이크(ADC+DMA) 기반 좌/우 방향 감지
- 팬/틸트 2축 서보 제어(TIM3 CH1/CH2)
- AHT10 온습도 센서(I2C1)
- UART2(터미널 디버그 + 1바이트 수동 명령)
- UART1(RPi 연동 바이너리 프레임 + JSON payload)
- FreeRTOS(CMSIS-RTOS v2) 기반 태스크 구동

## 1. 핀/주변장치
- `PA6` : `TIM3_CH1` (PAN 서보)
- `PA7` : `TIM3_CH2` (TILT 서보)
- `PB8` : `I2C1_SCL` (AHT10)
- `PB9` : `I2C1_SDA` (AHT10)
- `PA2/PA3` : `USART2` (터미널)
- `PA9/PA10` : `USART1` (RPi 프로토콜)
- `PB0/PB1/PB2` : IR LED GPIO 출력

## 2. 모듈 구성
- `Core/Src/mic.c`
  - DMA 버퍼(`L,R,L,R...`) 처리
  - 짧은창(`SIG_WIN`) + 긴창(`NOISE_WIN`) 기반 SNR 계산
  - 게이트(`SOUND_TH`, `SNR_TH_Q8`, `DIR_RATIO_TH_Q8`)로 방향 판정
- `Core/Src/motor_ctrl.c`
  - 자동 추적 상태머신
  - 수동 step 제어(`manual_move_pan`, `manual_move_tilt`)
  - 자동 진입 시 센터 복귀(`motor_ctrl_enter_auto`)
- `Core/Src/aht10.c`
  - 측정 주기 기본 2000ms, 변환 대기 80ms
  - `N=10` FIFO 이동평균 필터
- `Core/Src/app.c`
  - 시스템 모드(AUTO/MANUAL)
  - UART 명령 처리
  - UART1 JSON 프레임 파싱/응답 송신
- `Core/Src/ir_led.c`
  - IR LED 제어 분리 모듈

## 3. UART 프로토콜 (RPi <-> STM, USART1)

프레임 형식:
- `CMD(1byte) + LEN(4byte, big-endian) + PAYLOAD(JSON bytes)`

### 3.1 상태 요청/응답
- 요청: `CMD=0x05`, `LEN=0`
- 응답: `CMD=0x05`, `LEN>0`, JSON
  - 예시:
  - `{"tmp":25.34,"hum":48.12,"dir":"L","tilt":91}`

JSON 키:
- `tmp`: 온도(섭씨)
- `hum`: 습도(%RH)
- `dir`: 감지 방향(`L`/`R`/`-`)
- `tilt`: 틸트 각도(0~180 근사값)

### 3.2 모터 명령/ACK
- 요청: `CMD=0x04`, `LEN>0`, JSON
  - 예시: `{"motor":"w"}`, `{"motor":"auto"}`
- 허용 토큰:
  - `w/a/s/d`
  - `auto/manual`
  - `on/off`
  - `o/f`
- 응답: `CMD=0x04`, ACK JSON
  - 예시: `{"ok":1,"mode":"manual","cmd":"w"}`

## 4. UART2 명령 (터미널)
- `o`/`O`: AUTO 모드
- `f`/`F`: MANUAL 모드
- 수동 모드에서만:
  - `w/s`: 틸트 이동
  - `a/d`: 팬 이동

## 5. FreeRTOS 현재 상태
- `ControlTask`에서 `app_loop()` 주기 실행(`osDelay(5)`)
- `UartRxTask`, `SensorTask`는 현재 기본 템플릿(확장 예정)
- 주의: `osKernelStart()` 이후 `main()`의 `while(1)`은 실행되지 않음

## 6. 로그 출력
- AUTO 모드:
  - 마이크 ADC/신호, 방향, PAN/TILT 각도, 온습도 출력
- MANUAL 모드:
  - `PAN/TILT` 각도와 PWM 값 출력

## 7. 빌드/적용 팁
- `.ioc` 변경 후 코드 재생성 시 USER CODE 영역 밖 수정은 덮어써질 수 있습니다.
- RTOS 환경에서는 태스크 컨텍스트 지연은 `osDelay` 권장.
- `motor.c`, `servo_calc.c`는 커널 상태에 따라 `osDelay/HAL_Delay`를 선택하는 래퍼를 사용합니다.

## 8. 다음 권장 작업
- UART ISR 최소화(큐에 push만) + 파싱을 `UartRxTask`로 이동
- `SensorTask`/`UartRxTask` 실사용 로직 이관
- HAL Timebase를 SysTick에서 별도 TIM(예: TIM11)로 분리 검토
