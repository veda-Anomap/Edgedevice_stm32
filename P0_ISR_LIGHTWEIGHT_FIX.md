# P0-1 ISR 경량화 수정

## 배경 문제
- I2S DMA half/full 콜백(ISR)에서 `mic_on_i2s_rx_segment()`를 바로 호출해,
  언팩/DC 제거/노이즈 게이트/TDOA half-block 누적/레벨 처리까지 한 번에 수행하고 있었습니다.
- 이 경로는 계산량이 커서, 인터럽트 점유 시간이 길어지고 RTOS 제어 루프 지터를 유발할 수 있습니다.

## 적용한 수정
- ISR에서는 세그먼트 큐 적재만 수행하도록 분리했습니다.
- 실제 DSP/레벨 처리는 Task 컨텍스트에서 `mic_i2s_drain()`으로 처리하도록 옮겼습니다.

## 코드 변경 파일
- `Core/Src/mic.c`
  - I2S 세그먼트 큐 추가
    - `i2s_seg_queue`, `i2s_seg_q_head/tail/count`, `i2s_seg_q_drop`
  - ISR enqueue 함수 추가
    - `mic_i2s_seg_enqueue_isr()`
  - Task drain 함수 추가
    - `mic_i2s_drain(uint8_t max_segments)`
  - 콜백 변경
    - `mic_on_i2s_rx_half_complete()` / `mic_on_i2s_rx_complete()`에서 큐 적재만 수행
  - reset 시 큐 초기화
    - `mic_i2s_seg_queue_reset()` 호출
- `Core/Src/app.c`
  - `app_control_loop()` 초반에 `mic_i2s_drain(4U)` 호출 추가

## 기대 효과
- ISR 점유 시간 감소
- ControlTask/SensorTask 스케줄링 안정성 개선
- TDOA/모터 제어 주기 지터 완화

## 확인 방법
1. UART 로그에서 TDOA 출력의 주기 흔들림이 줄어드는지 확인
2. 제어 입력 burst 시 모터 응답이 덜 끊기는지 확인
3. 필요 시 `mic_i2s_drain(4U)`의 drain 개수를 2~6 범위에서 튜닝
