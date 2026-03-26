# TDOA 문제점 우선순위 리뷰 (현재 코드 기준)

목적
- `tdoa_analysis_report.md`의 지적사항을 현재 코드 기준으로 재평가
- 코드 버그와 시스템 물리 한계를 분리해서 우선순위화

우선순위 기준
- `P0`: 즉시 동작 불안정/품질 급락을 유발할 가능성 큼
- `P1`: 상황 의존적으로 품질 저하를 유발
- `P2`: 개선 권장 수준, 즉시 치명도 낮음

---

## P0 (즉시 대응)

### 1) I2S ISR 처리량 과다
- 파일: `Core/Src/mic.c`
- 라인: `472`, `531`
- 내용:
  - ISR 경로에서 언팩, DC 제거, TDOA half-block 축적, 레벨 처리까지 수행
  - ISR 실행 시간이 길어지면 RTOS 태스크(제어/통신) 타이밍을 직접 압박
- 판단: 실제 모터 제어 품질 저하로 이어질 수 있어 `P0`

### 2) TDOA/fallback 전환 구간의 모터 상태 튐
- 파일: `Core/Src/app.c`, `Core/Src/motor_ctrl.c`
- 라인: `app.c:785`, `motor_ctrl.c:131`
- 내용:
  - TDOA valid/invalid 경계에서 추종과 fallback이 교차되면 상태 리셋이 반복되어 튐 발생 가능
- 판단: 사용자 체감 이상 동작과 직결되어 `P0`

---

## P1 (중요 개선)

### 3) `tdoa_dbg` 구조체 비원자 복사
- 파일: `Core/Src/mic.c`, `Core/Src/app.c`
- 라인: `mic.c:75`, `mic.c:926`, `app.c:795`
- 내용:
  - 한 태스크가 쓰는 중 다른 태스크가 읽어 torn-read 가능
  - 주로 로그/판정 일관성 저하
- 판단: 즉시 치명은 아니지만 품질 저하 요인이라 `P1`

### 4) 소프트웨어 FFT 부하
- 파일: `Core/Src/mic.c`, `Core/Src/main.c`
- 라인: `mic.c:193`, `mic.c:624`, `main.c:747`
- 내용:
  - 20ms 주기에서 FFT/IFFT + 수학함수 수행
  - 다른 태스크 부하와 겹치면 지터 증가 가능
- 판단: 성능 여유를 줄이는 구조로 `P1`

---

## 시스템 물리 한계 (중요, 코드 버그와 별도 관리)

### A) 마이크 간격 120mm + 16kHz의 분해능 한계
- 파일: `Core/Src/mic.c`
- 라인: `43`, `39`, `47`, `736`
- 근거:
  - `d = 120mm`, `fs = 16kHz`에서 물리적 최대 지연은 약 `5.6 samples`
  - 실질 탐색 lag가 좁아 각도 분해능이 계단형으로 나타날 수 있음
- 의미:
  - 소프트웨어만으로 완전 해소 불가
  - 파라미터 튜닝 상한/하한을 정하는 “시스템 한계”로 반드시 인지 필요
- 우선순위 분류: `P1-제약` (버그는 아니지만 결과 품질 상한을 결정)

### B) 실내 반사음(Multipath) 취약성
- 파일: `Core/Src/mic.c`
- 라인: `798`(confidence), `719`(imbalance reject)
- 근거:
  - GCC-PHAT는 직접음 조건에서 강하지만, 실내 반사 환경에서는 spurious peak가 증가
  - confidence가 불안정해지면 valid 토글이 잦아지고 전환 구간 떨림(P0 #2)을 악화시킬 수 있음
- 의미:
  - 완전 제거는 어려우며, 배치/환경/하드웨어와 함께 관리해야 함
  - 튜닝(hold/hysteresis/confidence gate)과 설치 환경 최적화가 필요
- 우선순위 분류: `P1-제약`

---

## P2 (권장 개선)

### 5) seqlock 경로의 명시적 메모리 배리어 부재
- 파일: `Core/Src/mic.c`
- 라인: `526`, `659`
- 판단:
  - 이론적 위험은 있으나 현재 M4 단일코어/무캐시 조건에서 즉시 치명도는 낮음
  - `P2`

### 6) “ControlTask 스택 오버플로우” 지적은 과장
- 파일: `Core/Src/mic.c`
- 라인: `624`
- 판단:
  - 보고서에서 언급한 큰 배열 상당수가 함수 `static`으로 BSS에 위치
  - 스택 이슈보다는 전체 RAM 점유 이슈가 맞음
  - `P2`

### 7) Hann window 지연 초기화
- 파일: `Core/Src/mic.c`
- 라인: `174`
- 판단:
  - 최초 1회성 지연 이슈
  - `P2`

---

## 결론

1차 대응(즉시)
1. ISR 경량화 (`P0`)
2. TDOA/fallback 전환 안정화 (`P0`)

2차 대응(중요)
1. `tdoa_dbg` 스냅샷 원자성 강화 (`P1`)
2. FFT 경량화/CMSIS-DSP 전환 검토 (`P1`)

상시 관리(제약 인지)
1. 120mm/16kHz 분해능 상한 (`P1-제약`)
2. 실내 반사음에 따른 confidence 불안정 (`P1-제약`)

---

## 추가 검토 의견 (피드백 반영)

본 문서는 기존 우선순위 본문을 유지하고, 아래 내용을 추가로 기록한다.

### A. 전반 평가
- 기존 분류의 큰 방향은 타당하다.
- 특히 `코드 버그`와 `시스템 물리 한계`를 분리한 점은 유지한다.

### B. 항목별 재평가
- P0 #1 ISR 처리량 과다: 유지 (`P0`)
- P0 #2 TDOA/fallback 전환 튐: 유지 (`P0`)
- P1 #3 `tdoa_dbg` 비원자 복사: 유지 (`P1`)
- P1 #4 소프트웨어 FFT 부하: 유지 (`P1`)

### C. 쟁점 항목 (#5 메모리 배리어)
- 기존 문서에서는 `P2`로 두었으나, 예방적 관점에서는 `P1` 상향이 타당하다는 의견이 있음.
- 근거:
  - 현재 Debug(`-O0`)에서는 재현 가능성이 낮을 수 있음.
  - 하지만 Release 최적화/툴체인 변화 시 잠복 리스크가 남음.
  - `__DMB()` 추가 비용이 매우 작아 예방적 하드닝 효과가 큼.
- 결론:
  - 운영 리스크 관점: `P2`에 가까움
  - 코드 품질/재발 방지 관점: `P1` 상향 타당
  - 따라서 본 항목은 프로젝트 정책에 따라 `P1` 또는 `P2`로 선택 가능하나, 하드닝 자체는 권장.

### D. 물리 한계 항목 (#A, #B) 중요도
- 120mm/16kHz 분해능 한계, 실내 반사음 취약성은 계속 `P1-제약`으로 유지.
- 두 항목은 코드 버그가 아니지만, 튜닝 상한과 하드웨어 재설계 판단에 직접 영향을 주므로 문서에서 반드시 관리해야 한다.


---

## 실제 적용 수정사항 (P0 반영)

### P0-1 ISR 경량화 적용 완료

변경 배경
- I2S DMA ISR(half/full)에서 언팩/DC 제거/노이즈 게이트/TDOA 누적/레벨처리를 한 번에 수행하던 구조는 ISR 점유 시간을 늘려 RTOS 스케줄 지터를 유발할 수 있음.

적용 내용
- ISR에서는 세그먼트 큐 적재만 수행하도록 변경.
- 실제 신호처리(DSP/레벨/TDOA 누적)는 Task 컨텍스트에서 `mic_i2s_drain()`으로 수행.

코드 반영
- `Core/Src/mic.c`
  - I2S 세그먼트 큐 추가: `i2s_seg_queue`, `i2s_seg_q_head/tail/count`, `i2s_seg_q_drop`
  - ISR enqueue 함수 추가: `mic_i2s_seg_enqueue_isr()`
  - Task drain 함수 추가: `mic_i2s_drain(uint8_t max_segments)`
  - 콜백 변경: `mic_on_i2s_rx_half_complete()`, `mic_on_i2s_rx_complete()` -> enqueue only
  - reset 시 큐 초기화: `mic_i2s_seg_queue_reset()` 호출
- `Core/Src/app.c`
  - `app_control_loop()` 초반에 `mic_i2s_drain(4U)` 호출 추가

기대 효과
- ISR 체류 시간 감소
- 제어 루프 주기 안정화
- TDOA/모터 제어 지터 완화

---

### P0-2 TDOA↔Fallback 전환 안정화 적용 완료

변경 배경
- TDOA valid/invalid 경계에서 즉시 전환이 반복되면 소스 튐과 모터 떨림이 발생.
- TDOA 추종 중 old one-shot 상태를 과도하게 리셋하면 fallback 복귀 시 불안정이 커질 수 있음.

적용 내용
1. TDOA 손실 확인 카운터 도입
- `Core/Src/app.c`
  - `TDOA_LOST_CONFIRM_CYCLES` 추가(현재 3)
  - `tdoa_lost_confirm` 상태 변수 추가
  - TDOA가 약해져도 즉시 fallback 진입하지 않고 연속 손실 확인 후 진입

2. TDOA active 중 과도한 상태 리셋 제거
- `Core/Src/motor_ctrl.c`
  - `motor_ctrl_track_pan_tdoa()`에서 `motor_lock_until_ms = 0U`, `last_dir = '-'` 리셋 제거
  - `pending_dir`, `motor_running`만 정리하고 lock/last_dir은 유지

기대 효과
- `SRC:T` ↔ `SRC:D` 경계 전환 튐 감소
- fallback 복귀 시 모터 동작 연속성 개선
- 한쪽 소리원 유지 상황에서 좌/우/중앙 반복 스위칭 완화

---

## 관련 상세 문서
- `P0_ISR_LIGHTWEIGHT_FIX.md`
- `P0_TDOA_FALLBACK_STABILITY_FIX.md`

---

## 실제 적용 수정사항 (P1 반영)

### P1-1 Debug Snapshot 동기화 강화 적용 완료

변경 배경
- `tdoa_dbg`/`mic_debug`는 서로 다른 Task에서 갱신/조회되므로, 구조체/필드 스냅샷 시 타이밍에 따라 torn-read 가능성이 있음.

적용 내용
- `Core/Src/mic.c`
  - critical section 헬퍼 추가
    - `mic_enter_critical()`
    - `mic_exit_critical()`
  - `mic_get_debug()`를 critical section으로 보호
  - `mic_get_tdoa_debug()`를 critical section으로 보호

기대 효과
- 로그 필드 불일치 감소
- TDOA 진단값(`valid/lag/tau/angle/conf`) 읽기 일관성 향상

---

### P1-2 FFT 부하 완화 1차 적용 완료

변경 배경
- 현재 TDOA 경로는 소프트웨어 FFT/IFFT 기반이라, 저레벨 무의미 구간까지 매번 수행하면 ControlTask 부하가 커질 수 있음.

적용 내용
1. 저레벨 precheck 추가
- `Core/Src/mic.c`
  - `TDOA_PRECHECK_LEVEL_TH` 추가
  - 추적 비활성(`tdoa_track_valid==0`)이고 양 채널 레벨이 임계 미만이면
    FFT 경로 진입 전에 invalid 처리 후 즉시 return

2. TDOA half-block publish 순서 보강
- `Core/Src/mic.c`
  - `tdoa_ready_l/r` memcpy 후 `__DMB()` 추가
  - 그 다음 `tdoa_ready_pub_idx`, `tdoa_ready_seq` 갱신

기대 효과
- 무음/저레벨 구간에서 불필요한 FFT 연산 감소
- frame publish 순서 안정성 개선

---

## 관련 상세 문서
- `P1_TDOA_DEBUG_SYNC_FIX.md`
- `P1_TDOA_LOAD_SHEDDING_FIX.md`

---

## P1 추가 진행사항 (통합 기록, 분리 문서 미사용)

### P1-3 FFT 경량화/CMSIS-DSP 전환 검토 결과

검토 배경
- P1의 핵심 과제는 소프트웨어 FFT 경로의 부하를 낮추는 것.
- 이상적인 방향은 CMSIS-DSP(`arm_rfft_fast_f32`/`arm_cfft_f32`) 전환.

현 코드/프로젝트 상태 확인
- 현재 프로젝트 트리 기준 `Drivers/CMSIS/DSP` 디렉터리가 없음.
- 즉시 전환하려면 DSP 소스/헤더 추가 및 빌드 설정(인클루드/컴파일 유닛) 선행이 필요.

이번 턴에서 반영한 현실적 대응
- CMSIS-DSP "즉시 전환" 대신, 현 구조에서 안전한 1차 부하 완화(이미 반영):
  1) 저레벨 precheck로 FFT 진입 자체를 줄임
  2) publish 순서 배리어 보강으로 데이터 안정성 강화

남은 작업(다음 P1 확장)
1. CMSIS-DSP 도입 준비
- DSP 소스(Transform/ComplexMath/BasicMath) 추가
- `arm_math.h` 경로/매크로(`ARM_MATH_CM4`) 설정

2. 단계적 치환
- `tdoa_fft_real_windowed_128()` / `tdoa_ifft_phat_to_corr_abs()`를 CMSIS 기반으로 치환
- 기존 경로는 `#if` 스위치로 fallback 유지

3. 성능 검증 지표
- ControlTask 루프 주기 분산
- TDOA 처리 시간(최대/평균)
- 큐 드롭(`i2s_seg_q_drop`) 증가 여부

판단
- 현재는 "P1 즉시 안정화" 관점에서 타당한 상태.
- CMSIS-DSP 전환은 프로젝트 의존성 추가가 필요한 "P1 확장 단계"로 진행하는 것이 안전함.

---

메모
- 요청에 따라 이후 수정사항 기록은 본 문서(`TDOA_ISSUE_PRIORITY_REVIEW.md`)에만 계속 누적한다.


## P1-4 CMSIS-DSP FFT 경로 전환 적용 완료 (2026-03-26)

변경 배경
- P1 잔여 항목이었던 FFT 경로 CMSIS-DSP 전환을 실제 코드에 반영.
- 기존 소프트웨어 FFT/IFFT 대비 연산 부하와 수치 안정성을 개선하기 위함.

적용 내용
1. `Core/Src/mic.c`
- `arm_math.h` 기반으로 `arm_cfft_init_f32`, `arm_cfft_f32` 경로 활성화.
- 기존 소프트웨어 FFT/IFFT 함수는 fallback으로 유지.
- TDOA 초기화 단계에서 CFFT 인스턴스 초기화 실패 시 자동 fallback.

2. CMSIS-DSP 소스/헤더 반영
- `Drivers/CMSIS/DSP/Source/...` 최소 Transform/CommonTables 경로 추가.
- `Drivers/CMSIS/Include/arm_math.h` 및 `dsp/*` 헤더 추가.

3. 링크 안정성 보강
- IDE가 생성하는 `Debug/objects.list`에 의존하지 않도록,
  CMSIS-DSP Transform 구현을 `mic.c`에 포함되도록 구성.
- 즉, `cmsis_dsp_bundle.o` 링크 유무와 무관하게 `mic.o`만으로 필수 심볼 해결 가능.

빌드 검증
- 툴체인 경로 지정 후 `make -C Debug -j1 all` 빌드 성공 확인.
- `irrled.elf` 링크 성공, `arm-none-eabi-size` 출력 정상.

남은 리스크/주의
- 성능 검증은 실제 보드 환경에서 UART 로그 기준으로 추가 확인 필요:
  - `TV/CONF/TAU/LAG/LOC_A` 안정도
  - ControlTask 주기 흔들림 여부
  - fallback 진입 빈도
