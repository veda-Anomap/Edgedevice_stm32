# P1-1 TDOA Debug Snapshot 동기화 수정

## 배경
- `mic_tdoa_process()`는 ControlTask에서 `tdoa_dbg`를 갱신하고,
  `app_sensor_loop()`는 SensorTask에서 `mic_get_tdoa_debug()`로 읽습니다.
- 구조체를 필드 단위로 갱신/복사하면, 태스크 전환 타이밍에 따라 torn-read 가능성이 있습니다.

## 적용 내용
- `mic.c`에 짧은 critical section 헬퍼 추가
  - `mic_enter_critical()`
  - `mic_exit_critical()`
- 아래 스냅샷 API를 critical section으로 보호
  - `mic_get_tdoa_debug()`
  - `mic_get_debug()`

## 효과
- 로그/진단에서 필드 불일치 가능성 감소
- TDOA valid/angle/lag 값 읽기 일관성 향상

## 변경 파일
- `Core/Src/mic.c`
