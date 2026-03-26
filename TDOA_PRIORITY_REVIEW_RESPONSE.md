# TDOA 우선순위 리뷰에 대한 재검토 의견

> `TDOA_ISSUE_PRIORITY_REVIEW.md` (Codex 작성)에 대한 Antigravity의 의견서

---

## 전체 평가

Codex 리뷰는 **전반적으로 실용적이며 우선순위 판단이 적절**합니다.
특히 업데이트된 버전에서 "코드 버그"와 "시스템 물리 한계"를 분리한 구조는 실무 대응 순서를 세울 때 매우 유용합니다.

아래에 항목별로 동의/이의를 정리합니다.

---

## P0 항목: 전면 동의

### P0 #1. ISR 처리량 과다 — ✅ 동의

`mic_on_i2s_rx_segment()` (mic.c:472-531) 안에서 수행하는 작업이 과중합니다:

1. `inmp441_unpack_s24()` × N회 (24-bit → 16-bit 변환)
2. DC offset 추적 (`dc_offset_l/r` IIR)
3. Noise gate 적용
4. TDOA half-block 축적 + `memcpy` 발행
5. `mic_process_interleaved_u16()` → 링 버퍼 + SNR 계산

이 전체가 **DMA half/complete ISR** 안에서 실행됩니다. 16kHz 샘플레이트에서 DMA half 콜백이 수 ms 간격으로 발생하므로, ISR 체류 시간이 길어져 RTOS 태스크 스케줄링을 직접 압박합니다.

**권장 수정 방향:**
- ISR에서는 raw 버퍼 포인터만 플래그하고, 언팩/DC/게이트/레벨 처리는 전용 Task에서 수행
- 또는 DMA double buffer + ping-pong 방식으로 ISR을 플래그 설정만으로 경량화

### P0 #2. TDOA/fallback 전환 시 모터 튐 — ✅ 동의

`motor_ctrl_track_pan_tdoa()` (motor_ctrl.c:131-136)에서:
```c
pending_dir = '-';
motor_running = 0U;
motor_lock_until_ms = 0U;  // ← 핵심 문제
last_dir = '-';
```
TDOA 추적이 활성화될 때마다 detect_dir 상태를 **전부 리셋**합니다. TDOA valid → invalid로 전환되면 `lock_until_ms=0` 상태에서 detect_dir fallback이 **즉시** 모터를 움직여 방향 바운스가 발생합니다.

`app_control_loop()` (app.c:785-824)에서도 TDOA 강도(≥12°) 조건과 fallback 사이의 전환이 TDOA confidence 흔들림에 따라 매 10ms 주기로 왕복할 수 있습니다.

**권장 수정 방향:**
- TDOA → fallback 전환 시 `lock_until_ms`를 보존하거나 최소 쿨다운 적용
- 전환 hysteresis 추가 (예: TDOA invalid가 N회 연속이면 fallback 진입)

---

## P1 항목: 동의 + 보강 의견

### P1 #3. `tdoa_dbg` 비원자 복사 — ✅ 동의

`mic_tdoa_debug_t`(9개 필드, 약 24바이트) 구조체를 `ControlTask`에서 갱신하고 `SensorTask`에서 복사합니다. 두 태스크 우선순위가 같아서(`osPriorityNormal` vs `osPriorityBelowNormal`) preemption이 발생합니다.

torn-read 시 로그에 `lag=-3`인데 `alpha=+45°` 같은 모순 데이터가 나타날 수 있고, 경계 조건에서 `valid` 플래그와 `alpha_deg_x10`이 불일치하면 모터 제어 판정도 영향받습니다.

P1이 맞되, P0 #2와 결합되면 체감 악화가 커질 수 있습니다.

### P1 #4. 소프트웨어 FFT 부하 — ✅ 동의

128-pt FFT × 3 (L, R, IFFT) + `sqrtf()` × 128 + `cosf()/sinf()` twiddle factor가 20ms마다 수행됩니다.

CMSIS-DSP의 `arm_cfft_f32()`는 하드코딩 twiddle + ARM 최적화로 **3~5배 빠릅니다.** 전환 비용이 낮고 효과가 크므로 P1 대응 시 최우선 검토 대상입니다.

---

## P2 #5. 메모리 배리어 — ⚠️ P1로 상향 제안

Codex는 "M4 단일코어/무캐시이므로 P2"로 판단했으나, **이의를 제기**합니다.

### 논점: 하드웨어가 아니라 컴파일러가 문제

| 관점 | Codex 주장 | 내 반론 |
|------|-----------|---------|
| 하드웨어 reordering | M4 단일코어에서 거의 없음 | ✅ 동의 |
| 컴파일러 reordering | 언급 없음 | ❌ 이것이 실제 위험 |

핵심 코드 (mic.c:522-528):
```c
// ISR에서 실행
memcpy(tdoa_ready_l[wr], tdoa_acc_l, sizeof(tdoa_acc_l));  // (A) non-volatile 대상
memcpy(tdoa_ready_r[wr], tdoa_acc_r, sizeof(tdoa_acc_r));  // (B) non-volatile 대상
tdoa_ready_pub_idx = wr;                                     // (C) volatile
tdoa_ready_seq++;                                            // (D) volatile
```

- `tdoa_ready_l/r` 배열은 **volatile이 아닙니다**
- C 표준에서 volatile 쓰기(C, D)와 non-volatile 쓰기(A, B)는 **순서 보장이 없습니다**
- GCC `-O2`에서 (C)가 (A) 이전으로 reorder되면, Task 측에서 **복사 미완료 데이터**를 읽습니다
- 이 버그는 **간헐적**이고 **재현이 극도로 어렵습니다**

### 수정 비용 vs 리스크

```c
// 수정: 딱 1줄 추가
memcpy(tdoa_ready_l[wr], tdoa_acc_l, sizeof(tdoa_acc_l));
memcpy(tdoa_ready_r[wr], tdoa_acc_r, sizeof(tdoa_acc_r));
__DMB();                          // ← 추가
tdoa_ready_pub_idx = wr;
tdoa_ready_seq++;
```

- 수정 비용: **1줄, 런타임 오버헤드 ~1 cycle**
- 미수정 시 리스크: **간헐적 TDOA 데이터 손상 → 디버깅 수주 소요 가능**

**결론: P2로 두기에는 리스크/비용 비율이 너무 불리합니다. P1 권장.**

---

## P2 #6. 스택 오버플로우 지적 — ✅ Codex 반론 인정

원래 보고서(tdoa_analysis_report.md)에서 제가 "ControlTask 스택 오버플로우 위험"으로 제목을 달았으나:

- `mic_tdoa_process()` 내 큰 배열 6개는 **함수 내 `static`** → BSS 영역에 위치
- 스택에 올라가는 로컬 변수는 스칼라 위주로 크지 않음
- ControlTask 스택 1536B로 충분할 가능성이 높음

**Codex가 "스택 항목으로는 타당성 낮음"이라고 한 건 정확합니다.** 이슈의 본질은 "96KB RAM에서 static 배열 ~4KB 점유"이며, 이는 별도 항목(RAM budget 관리)으로 봐야 합니다. P2 동의.

## P2 #7. Hann 윈도우 지연 초기화 — ✅ 동의

최초 1회성이고 체감 영향이 없습니다. P2.

---

## 시스템 물리 한계: 동의 + 보강

### A) 120mm/16kHz 분해능 → P1-제약 ✅ 동의

유효 lag ±6 샘플에서 parabolic interpolation으로 보간해도, 각도 **분해능 하한이 약 ±13°**입니다. 이는 TDOA 파라미터(TDOA_DIR_ENTER_X10=90 → 9°)와 상충합니다.

> 참고: 샘플레이트를 32kHz나 48kHz로 올리면 분해능이 2~3배 개선되지만, FFT 부하도 비례 증가합니다. 하드웨어 재설계 시 검토 필요.

### B) 실내 반사음 취약성 → P1-제약 ✅ 동의

GCC-PHAT confidence 불안정이 P0 #2(모터 튐)를 악화시키는 구조적 연쇄입니다. 환경 최적화(마이크 배치, 흡음재)와 소프트웨어 holdoff 튜닝을 병행해야 합니다.

---

## 최종 요약

| # | 항목 | Codex 판정 | 내 판정 | 비고 |
|---|------|-----------|---------|------|
| 1 | ISR 처리량 과다 | P0 | **P0** ✅ | |
| 2 | TDOA/fallback 전환 튐 | P0 | **P0** ✅ | |
| 3 | `tdoa_dbg` 비원자 복사 | P1 | **P1** ✅ | |
| 4 | FFT 부하 | P1 | **P1** ✅ | |
| 5 | 메모리 배리어 부재 | P2 | **P1** ⚠️ | 컴파일러 reorder 위험, 수정 비용 1줄 |
| 6 | 스택 오버플로우 지적 | P2 | **P2** ✅ | 원래 보고서 표현 부정확 인정 |
| 7 | Hann 지연 초기화 | P2 | **P2** ✅ | |
| A | 120mm/16kHz 분해능 | P1-제약 | **P1-제약** ✅ | |
| B | 실내 반사음 | P1-제약 | **P1-제약** ✅ | |

**유일한 의견 차이: #5 메모리 배리어 (P2 → P1 상향 제안)**

나머지는 Codex 리뷰와 완전히 일치합니다.
