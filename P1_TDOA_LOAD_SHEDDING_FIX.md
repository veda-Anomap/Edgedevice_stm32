# P1-2 TDOA FFT 부하 완화 1차 수정

## 배경
- 현재 TDOA는 소프트웨어 FFT/IFFT 기반 GCC-PHAT를 사용하므로,
  무의미한 저레벨 구간까지 계속 연산하면 ControlTask 부하가 커질 수 있습니다.

## 적용 내용
1. 저레벨 precheck 추가
- `TDOA_PRECHECK_LEVEL_TH` 기준을 추가하고,
  추적 비활성 상태(`tdoa_track_valid==0`)에서 양 채널 레벨이 모두 낮으면
  FFT 단계로 진입하지 않고 즉시 invalid 처리 후 리턴.

2. TDOA half-block publish 배리어 보강
- `tdoa_ready_l/r` memcpy 후 `__DMB()`를 추가하고
  그 다음 `tdoa_ready_pub_idx`, `tdoa_ready_seq`를 갱신.

## 효과
- 무음/저레벨 구간에서 불필요한 FFT 연산 감소
- TDOA frame publish 순서 안정성 개선

## 변경 파일
- `Core/Src/mic.c`
