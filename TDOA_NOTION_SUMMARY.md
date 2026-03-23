# TDOA 정리 노트 (Notion용)

이 문서는 현재 펌웨어 코드 기준으로, 2채널 I2S 마이크의 TDOA(Time Difference of Arrival) 경로를
"동작 원리 + 파라미터 + 튜닝 포인트 + 디버그 해석" 관점에서 한 번에 정리한 실무용 문서입니다.

---

## 1. 목표와 범위

### 목표
- 기존 `L/R` 이진 방향 감지보다 정밀한 **좌우 각도(-90~+90도)** 추정.
- 추정 각도를 PAN 서보에 반영해 부드럽게 추종.
- 반향/잡음 환경에서도 오탐과 튐을 줄이는 안정화 로직 적용.

### 범위
- `Core/Src/mic.c`: TDOA 신호 처리, 추정, 안정화
- `Core/Src/app.c`: AUTO 제어 경로에서 TDOA 우선 적용
- `Core/Src/motor_ctrl.c`: 각도 -> PWM 매핑 및 추종 제어

---

## 2. 전체 파이프라인 (코드 기준)

1. I2S DMA 수신
2. 24-bit 샘플 언팩 -> 16-bit 스케일
3. 채널별 DC 오프셋 제거 + 노이즈 게이트
4. 64샘플 half-block 누적
5. 이전 64 + 현재 64를 합쳐 128 프레임(50% overlap) 구성
6. VAD(mean abs) 통과 시에만 TDOA 연산
7. GCC-PHAT 기반 lag 탐색
8. 포물선 보간으로 sub-sample 보정
9. tau(us) -> angle(deg) 변환
10. confidence hysteresis + hold + EMA + slew 제한
11. AUTO 모드에서 PAN 추종기로 전달

---

## 3. 수식/모델 핵심

### 3-1. 시간차 -> 각도
- `tau_max = d / c`
- `x = tau / tau_max`, `x`를 `[-1, 1]`로 clamp
- `alpha = asin(x)`

코드에서는 각도를 `deg*10` 정수로 다룸 (`alpha_deg_x10`).

### 3-2. GCC-PHAT
- 주파수 영역 교차 스펙트럼 `G = X1 * conj(X2)`
- PHAT 정규화: `Gphat = G / (|G| + eps)`
- lag별 상관값을 계산해 최대 피크 lag 선택

### 3-3. 서브샘플 보간
- 최대 피크의 좌/중/우 3점으로 포물선 보간
- `p = 0.5 * (a-c)/(a-2b+c)`, `p`는 `[-0.5, 0.5]`
- `lag_f = lag + p`

---

## 4. 현재 주요 파라미터 (코드 실값)

### 4-1. TDOA 추정 파라미터 (`mic.c`)
- `TDOA_HALF_N = 64`
- `TDOA_FRAME_N = 128` (50% overlap)
- `TDOA_FS_HZ = 16000`
- `TDOA_LAG_MAX = 8`
- `TDOA_MIC_DISTANCE_MM = 120.0`
- `TDOA_SOUND_SPEED_MM_S = 343000.0`
- `TDOA_VAD_MEANABS_TH = 380`
- `TDOA_PHAT_EPS = 1e-9`

### 4-2. 안정화/유효성 파라미터 (`mic.c`)
- `TDOA_CONF_ON_Q8 = 384`
- `TDOA_CONF_OFF_Q8 = 300`
- `TDOA_VALID_HOLD_MS = 120`
- `TDOA_EMA_ALPHA_Q8 = 64`
- `TDOA_MAX_STEP_DEG_X10 = 80`
- `TDOA_ANGLE_CLAMP_DEG_X10 = 900`

### 4-3. 연산 부하 제어 (`mic.c`)
- `TDOA_PROCESS_PERIOD_MS = 20`
- `TDOA_STALE_INVALID_MS = 300`

### 4-4. AUTO fallback (`app.c`)
- `TDOA_FALLBACK_HOLD_MS = 250`
- valid 소실 직후 짧은 hold 이후 기존 detect_dir 경로로 fallback

### 4-5. PAN 추종기 (`motor_ctrl.c`)
- `TDOA_STEP_MIN_PWM = 3`
- `TDOA_STEP_MAX_PWM = 18`
- `TDOA_DEADBAND_PWM = 4`
- `TDOA_GAIN_Q8 = 160`
- `TDOA_ACCEL_UP_PWM = 2`
- `TDOA_ACCEL_DN_PWM = 3`
- `TDOA_SIGN = 1` (좌우 부호 뒤집기용)

---

## 5. 모듈별 역할 요약

### 5-1. `mic.c`
- I2S 입력 정제(DC blocking/noise gate)
- TDOA 프레임 구성(64+64)
- VAD -> GCC-PHAT -> lag/tau/angle 추정
- confidence/hold/EMA/slew로 `tdoa_dbg.valid` 결정

### 5-2. `app.c`
- AUTO에서만 `mic_tdoa_process(now)` 실행
- `tdbg.valid=1`이면 `motor_ctrl_track_pan_tdoa()` 호출
- valid 끊기면 짧게 hold 후 기존 `mic_process()` + `motor_ctrl_process()`로 fallback
- 센서 로그에 `TDOA[...]` 포함 출력

### 5-3. `motor_ctrl.c`
- 각도(-90~+90) -> PAN PWM 선형 매핑
- deadband/가변 step/가감속으로 부드럽게 목표 추종
- TDOA 추종 중 기존 one-shot L/R 상태머신 영향 최소화

---

## 6. 런타임 로그 해석

AUTO 로그 예:

`TDOA[V:1 L:2 T:125us A:13.4deg C:420]`

- `V`: valid 여부 (0/1)
- `L`: 정수 lag 샘플
- `T`: 시간차(us)
- `A`: 추정 각도(deg)
- `C`: confidence(q8)

`SRC` 값:
- `SRC:T` -> TDOA 기반 제어 중
- `SRC:D` -> 기존 detect_dir fallback 중
- `SRC:M` -> manual 모드

---

## 7. 튜닝 순서 권장

1. **입력 안정화 먼저**
- DC 오프셋/노이즈 게이트 정상인지 확인
- 무음에서 `V=0`, 소리 이벤트에서만 `V=1` 되는지 확인

2. **VAD/유효성 조정**
- 민감도 낮음: `TDOA_VAD_MEANABS_TH` 감소
- 오탐 많음: `TDOA_VAD_MEANABS_TH` 증가

3. **confidence/hold 조정**
- valid 깜빡임: `CONF_ON` 소폭 하향 또는 `VALID_HOLD_MS` 증가
- 반사 오탐: `CONF_ON` 상향, `CONF_OFF` 상향

4. **움직임 품질 조정**
- 느림: `STEP_MAX`, `GAIN_Q8` 상승
- 급격함/튐: `STEP_MAX` 하향, `ACCEL_UP` 하향, `EMA_ALPHA` 하향
- 소폭 떨림: `DEADBAND` 소폭 증가

---

## 8. 알려진 한계/주의사항

- 현재 DFT는 128점 직접 계산 방식이라 CPU 부담이 큼.
  - 추후 CMSIS-DSP FFT로 교체하면 부하 개선 가능.
- 2마이크 1D 배열 특성상 좌우 각도 추정만 가능(상하/거리 직접 추정 불가).
- 실내 반향 환경에서 피크 경쟁이 커지면 confidence 기준 조정 필요.
- `TDOA_SIGN`은 실제 배선 좌우와 반대일 때 즉시 수정 포인트.

---

## 9. 빠른 점검 체크리스트

- [ ] AUTO 모드에서만 TDOA 연산 수행되는가
- [ ] 무음에서 `V=0`, 이벤트에서 `V=1`이 되는가
- [ ] `A` 부호가 실제 좌/우와 일치하는가 (`TDOA_SIGN` 확인)
- [ ] valid 소실 후 `SRC:T -> SRC:D` 전환이 정상인가
- [ ] PAN 이동이 deadband/가감속 규칙대로 부드러운가

---

## 10. 참고 파일

- `Core/Inc/mic.h`
- `Core/Src/mic.c`
- `Core/Src/app.c`
- `Core/Inc/motor_ctrl.h`
- `Core/Src/motor_ctrl.c`
- `TDOA_CHANGELOG_DETAILED.md`

