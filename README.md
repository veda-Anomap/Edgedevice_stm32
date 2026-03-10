# EdgeDevice STM32 (NUCLEO-F401RE)

STM32F401RE 기반 엣지 디바이스 펌웨어 프로젝트입니다.

## 1. 현재 구현 범위
- 듀얼 마이크(ADC + DMA) 기반 좌/우 방향 감지
- 2축 서보 제어(PAN/TILT, TIM3 CH1/CH2)
- AHT10 온습도 센서(I2C1)
- PCF8591 조도 센서(I2C1, CH0)
- UART1: RPi 연동용 바이너리 프레임 + JSON payload
- UART2: 터미널 디버그/수동 테스트 명령
- FreeRTOS(CMSIS-RTOS v2) 기반 태스크 분리

## 2. 최근 변경 사항 (2026-03-10)
- PCF8591 조도센서 모듈 추가
  - `Core/Inc/pcf8591.h`
  - `Core/Src/pcf8591.c`
- 상태 응답 JSON(`CMD=0x05`)에 `light` 필드 추가
  - `{"tmp":..,"hum":..,"dir":"..","tilt":..,"light":..}`
- UART1 모터 명령 JSON 허용 토큰 정리
  - 허용: `w`, `a`, `s`, `d`, `auto`, `unauto`
  - 미허용: `manual`, `on`, `off`, `o`, `f` (UART1 JSON 경로)
- UART2 단문 테스트(`o/f/w/a/s/d`)는 기존 유지

## 3. 핀 매핑
- `PA6`: `TIM3_CH1` (PAN 서보)
- `PA7`: `TIM3_CH2` (TILT 서보)
- `PB8`: `I2C1_SCL` (AHT10 + PCF8591 공용)
- `PB9`: `I2C1_SDA` (AHT10 + PCF8591 공용)
- `PA9/PA10`: `USART1` (RPi 프로토콜)
- `PA2/PA3`: `USART2` (터미널)
- `PB0/PB1/PB2`: IR LED GPIO 출력

## 4. 모듈 구성
- `Core/Src/mic.c`
  - DMA 인터리브 샘플(L,R,L,R...) 처리
  - 신호/노이즈 윈도우 기반 SNR 계산
  - 임계값 게이트 기반 방향 판정
- `Core/Src/motor_ctrl.c`
  - 자동 추적 상태머신
  - 수동 step 제어(`manual_move_pan`, `manual_move_tilt`)
- `Core/Src/aht10.c`
  - 2초 주기 측정, 변환 대기시간 반영
  - 이동평균 필터 적용
- `Core/Src/pcf8591.c`
  - CH0 조도(8-bit 원시값) 주기 샘플링
  - 더미 바이트 폐기 후 실제 값 사용
- `Core/Src/app.c`
  - AUTO/MANUAL 모드 관리
  - UART1 프로토콜 파싱/응답
  - 센서 루프/제어 루프 분리

## 5. UART1 프로토콜 (RPi <-> STM)
프레임 형식:
- `CMD(1byte) + LEN(4byte, big-endian) + PAYLOAD(JSON bytes)`

### 5.1 상태 요청/응답
- 요청: `CMD=0x05`, `LEN=0`
- 응답: `CMD=0x05`, `LEN>0`, JSON
  - 예시: `{"tmp":25.34,"hum":48.12,"dir":"L","tilt":91,"light":123}`

필드 설명:
- `tmp`: 온도(섭씨)
- `hum`: 습도(%RH)
- `dir`: 감지 방향(`L`/`R`/`-`)
- `tilt`: 틸트 각도(도)
- `light`: 조도 원시값(0~255)

### 5.2 모터 명령/ACK
- 요청: `CMD=0x04`, `LEN>0`, JSON
  - 예시: `{"motor":"w"}`, `{"motor":"auto"}`
- 허용 토큰:
  - `w/a/s/d`
  - `auto/unauto`
- 응답: `CMD=0x04`, ACK JSON
  - 예시: `{"ok":1,"mode":"manual","cmd":"w"}`

## 6. UART2 단문 명령 (터미널)
- `o`/`O`: AUTO 모드
- `f`/`F`: MANUAL 모드
- MANUAL 모드에서:
  - `w/s`: 틸트 이동
  - `a/d`: 팬 이동

## 7. FreeRTOS 태스크 구성
- `UartRxTask`
  - UART1 바이트 수신 큐 처리
  - 프레임 조립/파싱
- `ControlTask` (`osDelay(10)`)
  - 모드/마이크/모터 제어
- `SensorTask` (`osDelay(20)`)
  - AHT10/PCF8591 주기 처리
  - 상태 로그 갱신

## 8. 로그 출력
- AUTO 모드:
  - 마이크 ADC/최종 신호, 방향, PAN/TILT, 온습도, 조도 출력
- MANUAL 모드:
  - PAN/TILT 각도와 PWM, 조도 출력

## 9. 빌드/운용 참고
- `.ioc` 재생성 시 USER CODE 영역 외 변경사항은 덮어쓸 수 있습니다.
- RTOS 환경 지연은 `osDelay` 사용을 권장합니다.
- I2C 센서 추가 시 주소 충돌과 전압 레벨(3.3V) 호환을 확인하세요.
