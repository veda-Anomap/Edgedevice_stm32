# 마이크 처리 상세 변경 이력 (TDOA 제외/기본 경로 중심)

이 문서는 **마이크 입력 파이프라인(ADC -> I2S 전환, 신호 안정화, 방향 게이트)** 중심의
변경 내역을 정리한 문서입니다.

- TDOA 단계별 상세는 별도 문서:
  - `TDOA_CHANGELOG_DETAILED.md`

---

## 0) 기본 방향

- 하드웨어 입력: INMP441(I2S)
- 목표:
  - 실환경 노이즈/오프셋 영향을 줄인 안정적인 방향 검출
  - 기존 모터/앱 로직과의 호환 유지
  - ADC 경로는 fallback로 유지

---

## 1) 핵심 변경 타임라인

## [v0.9.0] - 구조 분리(마이크/모터/앱)

### 변경 전
- 기능이 단일 파일에 몰려 변경 영향 파악이 어려움

### 변경
- `mic.c`를 마이크 전용 처리 책임으로 분리
- `app.c`에서 연계/모드 제어 통합

### 효과
- 마이크 쪽 수정 시 회귀 범위가 명확해짐.

---

## [v1.0.0] - ADC -> I2S 전환 + 신호 안정화

### 변경 전
- ADC 기반 코드와 I2S 전환 코드 혼재
- 기준점/스케일/노이즈 처리 불명확

### 변경
- I2S 수신 데이터 언팩(24-bit signed) 경로 정리
- 동적 DC 오프셋 추적(DC blocking) 적용
- 노이즈 게이트 적용
- short/long window 기반 신호/노이즈 분리 유지
- ADC 경로는 조건부 컴파일 fallback 유지

### 효과
- I2S 환경에서 오탐 감소, 기존 상위 제어 로직과의 호환 확보.

---

## [신호 게이트 단순화] - 방향 판정 임계값 정리

### 배경
- 과도한 다중 게이트는 현장 튜닝 난이도를 크게 높임

### 변경 방향
- 핵심 게이트 중심으로 단순화:
  - `SOUND_TH`
  - `SNR_TH_Q8`
  - `DIR_RATIO_TH_Q8`
- 디버그 출력으로 gate 통과 상태를 즉시 확인 가능하게 구성

### 효과
- 튜닝 포인트 축소, 현장 조정 시간 감소.

---

## [유지/보강된 안정화 요소]

### 1) DC 오프셋 추적
- `dc_offset_l/r`를 느리게 추적해 중심축 드리프트를 흡수

### 2) 노이즈 게이트
- 미세 진동/바닥 잡음을 0으로 처리해 잔떨림 완화

### 3) 이중 윈도우 기반 SNR
- short window(최근 신호), long window(배경 노이즈) 분리
- `snrL_q8/snrR_q8` 기반 방향 게이트

### 4) 홀드오프/락
- 방향 전환 최소 간격(`SWITCH_HOLDOFF`)으로 빠른 뒤집힘 억제

---

## 2) 현재 마이크 기본 처리 흐름

1. I2S DMA 수신 (`HAL_I2S_RxCpltCallback -> mic_on_i2s_rx_complete`)
2. INMP441 샘플 언팩(24-bit signed -> 내부 처리 스케일)
3. DC blocking + noise gate
4. interleaved 처리로 채널별 frame magnitude 계산
5. short/long 윈도우 업데이트
6. SNR/비율 게이트 계산
7. `detect_dir` 결정 (조건 충족 시 L/R 갱신)

---

## 3) 주요 튜닝 파라미터(기본 경로)

- 감도:
  - `SOUND_TH`
- 신호/잡음:
  - `SIG_WIN`, `NOISE_WIN`
  - `SNR_TH_Q8`
  - `NOISE_FREEZE_Q8`
- 방향 우세:
  - `DIR_RATIO_TH_Q8`
- 전환 안정화:
  - `SWITCH_HOLDOFF`
- 전처리:
  - `DC_TRACK_SHIFT`
  - `NOISE_GATE_TH`

---

## 4) 진단/로그 관찰 포인트

- 원시/중간값:
  - `I2S_L`, `I2S_R`
  - `FINAL_L`, `FINAL_R`
- 게이트:
  - `gate_sound`, `gate_snr`, `gate_ratio`
- 결과:
  - `DIR`

권장 확인 시나리오
- 무입력 2분 오탐 횟수
- 좌/우 단발 박수 20회 오판정률
- 소음 환경(팬/대화)에서 방향 안정성

---

## 5) 관련 파일

- `Core/Inc/mic.h`
- `Core/Src/mic.c`
- `Core/Src/app.c`
- `Core/Src/main.c`
- `irrled.ioc`
- `VERSION_HISTORY.md`

