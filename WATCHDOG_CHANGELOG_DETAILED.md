# 워치독 상세 변경 이력 (외부 릴레이 + 내부 IWDG)

이 문서는 STM32 프로젝트의 워치독 체계 변경을
**중간 수정 포함**으로 단계별 정리한 전용 문서입니다.

- 외부 워치독: RPi heartbeat(PC3) 감시 + 릴레이(PC10) 제어
- 내부 워치독: IWDG refresh 모니터링

---

## 0) 최종 구조 요약

- `rpi_watchdog_task()`
  - heartbeat edge 감시
  - timeout 시 relay cut/restore
  - grace 재대기 후 재arm
- `sys_watchdog_task()`
  - `rpi_watchdog` alive counter 증가 여부 확인
  - miss 누적 시 IWDG refresh 중단 -> 하드웨어 리셋 유도

핵심 파일:
- `Core/Src/rpi_watchdog.c`, `Core/Inc/rpi_watchdog.h`
- `Core/Src/sys_watchdog.c`, `Core/Inc/sys_watchdog.h`
- 초기화/태스크 연결: `Core/Src/main.c`

---

## 1) 단계별 상세 타임라인

## [v1.2.1] - 외부/내부 워치독 최종 튜닝 + 큐 복구

### 변경 전
- IWDG timeout 대비 miss 임계가 타이트해서 지터에 취약
- `.ioc` 재생성으로 `control_queue`가 64 -> 16 축소

### 변경
- `SYS_MONITOR_MISS_MAX=4`로 조정
- grace/relay 대기 helper 정리:
  - `rpi_wdg_wait_seconds_and_mark_alive()`
  - `rpi_wdg_sync_heartbeat_baseline()`
- `control_queue` 64 복구(코드/IOC 일치)

### 효과
- 워치독 오탐 리셋 여유 증가
- burst 명령 수용성 회복

---

## [v1.2.2] - 리셋 원인 로깅 + IWDG 스케줄링 보강

### 변경 전
- 반복 리셋 시 원인(IWDG/BOR) 분리 어려움
- monitor 태스크 우선순위가 낮아 refresh 밀림 우려

### 변경
- 부팅 시 reset cause 로그 출력:
  - `BOR/PIN/POR/SW/IWDG/WWDG/LPWR`
- 출력 후 reset flags clear
- IWDG 설정 로그 출력(`presc/reload/miss_max`)
- 태스크 우선순위 재배치:
  - SystemMonitorTask 상향
  - UartRxTask 조정

### 효과
- 리셋 원인 판별 속도 개선
- 부하 상황에서 IWDG refresh 안정성 개선

---

## [v1.2.3] - 워치독 태스크 스택 증설(512B -> 1536B)

### 변경 전
- `[RST]`, `[IWDG]` 후 로그 절단/즉시 재부팅 반복
- `IWDG:1` 패턴 지속

### 변경
- `defaultTask` 스택:
  - 512B -> 1536B
- 로그 포맷/전송 경로 스택 마진 확보

### 효과
- 부팅 직후 재부팅 빈도 완화
- 워치독 루프 진입 안정성 개선

---

## [v1.2.4] - 첫 heartbeat 전 timeout 비활성(hb arm)

### 변경 전
- grace 직후 timeout 감시 시작으로, RPi heartbeat 앱 늦게 시작하면 오탐 릴레이 컷 발생

### 변경
- `hb_armed` 도입
- 첫 edge 감지 전 timeout 비활성
- 첫 edge 수신 시 arm 전환
- 상태 로그 추가:
  - `waiting first edge`
  - `watchdog armed`

### 효과
- “처음부터 안 오던 heartbeat”에 대한 오탐 컷 제거
- “오다가 끊김” 케이스에만 동작

---

## [v1.2.5] - 워치독 코드 분리(main.c -> 전용 모듈)

### 변경 전
- 외부/내부 워치독이 `main.c`에 섞여 유지보수/테스트 어려움

### 변경
- 파일 분리:
  - `rpi_watchdog.*`
  - `sys_watchdog.*`
- `main.c`는 init + 태스크 래퍼만 유지

### 효과
- 책임 분리/가독성 향상
- 외부/내부 워치독 독립 디버깅 쉬워짐

---

## 2) 현재 코드 기준 정책/파라미터

## 외부(RPi) 워치독
- 기본값:
  - `grace_sec=60`
  - `heartbeat_timeout_ms=10000`
  - `relay_cut_sec=3`
  - `poll_ms=10`
- 정상 상태:
  - `RELAY_EN=LOW`
- timeout:
  - `RELAY_EN=HIGH` (cut)
  - 3초 대기
  - `RELAY_EN=LOW` (restore)
  - grace 재대기
  - `hb_armed=0`으로 재시작

## 내부(IWDG) 모니터
- 주기:
  - `period_ms=1000`
- miss 임계:
  - `miss_max=4`
- 동작:
  - alive counter 변화 있음 -> refresh
  - 변화 없음 -> miss 누적
  - 임계 도달 후 refresh 중단해 HW reset 유도

---

## 3) 부팅/운영 로그 해석 가이드

- `[RST] ... IWDG:1 ...`
  - 직전 리셋 원인이 IWDG였음을 의미
- `[IWDG] presc=... reload=... miss_max=...`
  - 현재 watchdog 설정 확인
- `[RPI-WDG] start: grace=... timeout=... cut=...`
  - 외부 워치독 정책 시작 로그
- `[RPI-HB] waiting first edge`
  - 첫 heartbeat 전, timeout 비활성 상태
- `[RPI-HB] first edge detected -> watchdog armed`
  - 이후부터 timeout 감시 활성
- `[RPI-HB] timeout=... -> link=LOST, relay cut`
  - heartbeat 단절로 릴레이 컷 수행

---

## 4) 실제 장애 대응 체크리스트

1. 반복 리셋 시 `[RST]` 플래그로 원인부터 분리(IWDG vs BOR)
2. heartbeat 라인(PC3) edge 로그 유무 확인
3. `waiting first edge`에서 멈추면 RPi heartbeat 앱/배선 문제 우선 점검
4. timeout 빈발 시
   - RPi heartbeat 주기/지연
   - timeout/grace 값
   - 릴레이 모듈 active level
5. IWDG 리셋만 반복 시
   - monitor task 우선순위/스택
   - 과도한 blocking 호출 여부

---

## 5) 관련 파일

- `Core/Src/rpi_watchdog.c`
- `Core/Inc/rpi_watchdog.h`
- `Core/Src/sys_watchdog.c`
- `Core/Inc/sys_watchdog.h`
- `Core/Src/main.c`
- `VERSION_HISTORY.md`
- `README.md`

