# TDOA 상세 변경 이력

이 문서는 프로젝트에서 **TDOA(1D 좌우 각도 추정)**를 단계적으로 붙여온 과정을
버전 순서대로 상세 정리한 전용 기록입니다.

---

## 0) 목표와 범위

- 목표:
  - 기존 `L/R` 이진 방향 제어를 넘어, 좌우 각도(`-90~+90`) 기반 PAN 추종으로 확장
  - 실내 반향/잡음 환경에서 유효 신호만 통과시키는 안정화 체계 확보
- 범위:
  - `mic.c` TDOA 추정 파이프라인
  - `app.c` AUTO 제어 경로 연동
  - `motor_ctrl.c` PAN 추종기(부드러운 모터 동작)

---

## 1) 단계별 상세 타임라인

## [v1.2.6] - TDOA 1단계: 스켈레톤 API/디버그 경로

### 변경 전
- TDOA 계산을 붙일 공용 API/디버그 컨테이너가 없어, 이후 확장 시 인터페이스 충돌 위험이 큼.

### 변경
- `Core/Inc/mic.h`
  - `mic_tdoa_debug_t` 추가
  - API 선언:
    - `mic_tdoa_enable()`
    - `mic_tdoa_process()`
    - `mic_tdoa_is_valid()`
    - `mic_get_tdoa_debug()`
- `Core/Src/mic.c`
  - 위 API 스텁 구현
  - 런타임 상태(`tdoa_enabled`, `tdoa_dbg`) 추가

### 효과
- 기존 동작 영향 없이 TDOA 증분 개발 가능 기반 확보.

---

## [v1.2.7] - TDOA 2단계: VAD + 50% 오버랩 + 코스 lag

### 변경 전
- 프레임 경계에서 이벤트(박수 등) 누락 가능
- 무의미 구간까지 동일 처리

### 변경
- `TDOA_HALF_N=64`, `TDOA_FRAME_N=128` 구성
- I2S 콜백에서 clean 샘플을 half-block으로 축적 후 ready 시퀀스 생성
- `mic_tdoa_process()`에서:
  - 이전 64 + 현재 64 => 128 프레임
  - mean-abs VAD(`TDOA_VAD_MEANABS_TH`)
  - 코스 lag/confidence 계산
  - `TDOA[V/L/C]` 디버그 로그 노출

### 효과
- 프레임 경계 누락 감소, 저에너지 구간 필터링.

---

## [v1.2.8] - TDOA 3단계: 경량 GCC-PHAT + 서브샘플 보간

### 변경 전
- 코스 lag만으로는 각도 해상도 부족
- 반향 환경에서 피크 분리 약함

### 변경
- 128프레임 DFT 양채널 계산
- cross-power PHAT 정규화
- 제한 lag 탐색
- 3점 포물선 보간으로 sub-sample 오프셋 `p` 계산
- `tau_us`, `alpha_deg_x10` 산출
- 로그 확장: `TDOA[V/L/T/A/C]`

### 효과
- 샘플 양자화 오차 완화, 각도 분해능 및 피크 안정성 개선.

---

## [v1.2.9] - TDOA 4단계: 히스테리시스 + hold + EMA + slew

### 변경 전
- confidence 경계에서 `valid` 깜빡임
- 프레임 단위 각도 튐

### 변경
- confidence hysteresis
  - ON: `TDOA_CONF_ON_Q8`
  - OFF: `TDOA_CONF_OFF_Q8`
- hold: `TDOA_VALID_HOLD_MS`
- 안정화 필터:
  - step 제한: `TDOA_MAX_STEP_DEG_X10`
  - EMA: `TDOA_EMA_ALPHA_Q8`
  - clamp: `TDOA_ANGLE_CLAMP_DEG_X10`

### 효과
- 경계 구간에서 valid 안정화, 급반전/미세 떨림 완화.

---

## [v1.2.10] - TDOA 5단계: AUTO PAN 제어 직접 연동 + fallback

### 변경 전
- TDOA 값은 계산되지만 실제 AUTO 모터는 `L/R` 위주

### 변경
- `motor_ctrl_track_pan_tdoa()` API 추가
- 각도(-90~+90) -> PAN PWM 매핑
- `app_control_loop()`:
  - valid면 TDOA 추종(`SRC:T`)
  - 신호 소실 시 짧은 hold 후 기존 `detect_dir`로 fallback(`SRC:D`)

### 효과
- 이진 제어에서 연속 각도 추종으로 확장.

---

## [v1.2.11] - TDOA 6단계: PAN 추종기 가변 이득/가감속

### 변경 전
- 추종 step이 고정에 가까워 큰 오차에서 느리고, 근접 구간 미세 떨림 체감

### 변경
- 파라미터 도입:
  - `TDOA_STEP_MIN/MAX_PWM`
  - `TDOA_GAIN_Q8`
  - `TDOA_ACCEL_UP/DN_PWM`
  - `TDOA_DEADBAND_PWM`
- 속도 상태 `tdoa_track_speed_pwm` 추가
- 오버슈트 방지(목표 교차 clamp)

### 효과
- 멀 때 빠르게, 가까우면 부드럽게 추종.

---

## [v1.2.12] - TDOA 7단계: 물리 lag 제한 + Hann window

### 변경 전
- lag 탐색 범위 고정(`±8`)으로 물리적으로 불가능한 지연 후보 포함
- 스펙트럼 누설로 가짜 피크 영향 가능

### 변경
- `tdoa_effective_lag_max()`:
  - `lag_max = round(fs*d/c) + margin`
  - `TDOA_LAG_MAX` 내로 clamp
- DFT 입력에 Hann 적용:
  - `tdoa_prepare_hann()`
  - `tdoa_dft_real_128(..., win, ...)`

### 효과
- 오탐 후보 감소, 피크 대조도 개선.

---

## [v1.2.13] - TDOA 8단계: 연산 스케줄링 최적화

### 변경 전
- 무거운 GCC-PHAT 연산이 과도하게 자주 호출
- 수동 모드에서도 불필요한 연산 수행

### 변경
- `mic.c`
  - `TDOA_PROCESS_PERIOD_MS=20`
  - `TDOA_STALE_INVALID_MS=300`
  - 20ms 미만 재호출은 heavy path 스킵
  - 결과 stale 시 `valid` 자동 해제
- `app.c`
  - `MODE_AUTO`에서만 `mic_tdoa_process()` 호출

### 효과
- CPU 점유 감소, MANUAL 응답성 개선, stale 결과 잔존 완화.

---

## 2) 현재 TDOA 처리 흐름(요약)

1. I2S DMA 수신 -> DC blocking + noise gate
2. 64샘플 half block 누적
3. 128프레임(50% overlap) 구성
4. VAD 통과 시에만 GCC-PHAT 경로 진입
5. lag 탐색(물리 제한) + sub-sample 보간
6. confidence hysteresis/hold + EMA/slew
7. AUTO에서 PAN 추종기로 반영, 소실 시 fallback

---

## 3) 핵심 튜닝 포인트

- 감도:
  - `TDOA_VAD_MEANABS_TH`
- 안정성:
  - `TDOA_CONF_ON_Q8`, `TDOA_CONF_OFF_Q8`, `TDOA_VALID_HOLD_MS`
- 부드러움:
  - `TDOA_MAX_STEP_DEG_X10`, `TDOA_EMA_ALPHA_Q8`
  - `TDOA_STEP_MIN/MAX_PWM`, `TDOA_ACCEL_UP/DN_PWM`
- 부하:
  - `TDOA_PROCESS_PERIOD_MS`

---

## 4) 관련 파일

- `Core/Inc/mic.h`
- `Core/Src/mic.c`
- `Core/Inc/motor_ctrl.h`
- `Core/Src/motor_ctrl.c`
- `Core/Src/app.c`
- `README.md`
- `VERSION_HISTORY.md`

