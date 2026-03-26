# P0-2 TDOA↔Fallback 전환 안정화 수정

## 배경 문제
- TDOA valid/invalid 경계에서 소스 전환이 빠르게 반복되면,
  모터가 TDOA 추종과 detect_dir fallback 사이를 튀면서 떨림이 발생할 수 있습니다.
- 특히 `motor_ctrl_track_pan_tdoa()`에서 old one-shot 상태를 과하게 리셋하면,
  fallback 복귀 시 lock/방향 상태가 끊겨 불안정이 커질 수 있습니다.

## 적용한 수정
1. TDOA 손실 확인 카운터 도입
- `Core/Src/app.c`
  - `TDOA_LOST_CONFIRM_CYCLES` 추가 (현재 3)
  - `tdoa_lost_confirm` 상태 변수 추가
  - TDOA가 즉시 약해져도 바로 fallback으로 넘어가지 않고,
    연속 확인 후 fallback 진입하도록 변경

2. TDOA active 중 과도한 상태 리셋 제거
- `Core/Src/motor_ctrl.c`
  - `motor_ctrl_track_pan_tdoa()`에서
    `motor_lock_until_ms = 0U`, `last_dir = '-'` 리셋 제거
  - pending/motor_running만 정리하고 lock/last_dir은 유지

## 코드 변경 파일
- `Core/Src/app.c`
- `Core/Src/motor_ctrl.c`

## 기대 효과
- TDOA confidence 경계에서 소스 튐 감소
- fallback 복귀 시 모터 동작 연속성 개선
- 한쪽 소리원 유지 시 좌/우/중앙 반복 스위칭 완화

## 확인 방법
1. 로그에서 `SRC:T`↔`SRC:D` 전환 빈도 확인
2. 동일 위치 소리원에서 `TDIR`과 PAN이 이전보다 덜 흔들리는지 확인
3. 필요 시 `TDOA_LOST_CONFIRM_CYCLES`를 2~5 범위로 조정
