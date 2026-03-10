# Version History

?꾨줈?앺듃 蹂寃??대젰?낅땲??  
?좎쭨 ?뺤떇: `YYYY-MM-DD`

---

## [v0.9.6] - 2026-03-10
### PCF8591 조도센서 추가 + 상태 JSON 확장
- 대상 파일:
  - Core/Inc/pcf8591.h (신규)
  - Core/Src/pcf8591.c (신규)
  - Core/Src/app.c
  - Debug/Core/Src/subdir.mk (로컬 빌드 항목 반영)
- 구현 내용:
  - I2C1 버스에 PCF8591(CH0) 주기 샘플링 모듈 추가
  - PCF8591 읽기 시퀀스 적용
    - Control byte 전송 (0x40, CH0)
    - 2바이트 수신 후 첫 바이트(dummy) 폐기, 두 번째 바이트를 조도값으로 사용
  - app_init()에서 pcf8591_init(&hi2c1, 200U) 추가
  - app_sensor_loop()에서 pcf8591_process(now) 주기 호출 추가

### UART1 상태 응답(JSON) 변경
- 대상 파일: Core/Src/app.c
- 변경 전 (CMD=0x05 응답):
  - {"tmp":..,"hum":..,"dir":"..","tilt":..}
- 변경 후 (CMD=0x05 응답):
  - {"tmp":..,"hum":..,"dir":"..","tilt":..,"light":..}
- 비고:
  - light는 PCF8591 CH0의 8-bit 원시값(0~255)

### UART1 모터 JSON 허용 토큰 정리
- 대상 파일: Core/Src/app.c (parse_motor_command)
- 허용 토큰(0x04 JSON):
  - w, a, s, d, auto, unauto
- 미허용 토큰:
  - manual, on, off, o, f
- 비고:
  - UART2 단문 제어(o/f/w/a/s/d)는 기존 유지

---
## [v0.9.5] - 2026-03-05
### v0.9.4 ?곸꽭 蹂닿컯 湲곕줉 (Dead Code ?뺣━ + TIM11 Timebase 遺꾨━)
- ????ぉ? v0.9.4??援ы쁽 ?섎룄/?숈옉 ?먮━瑜????먯꽭???④릿 蹂닿컯 ?대젰?낅땲??

### A) Dead Code ?뺣━ ?곸꽭 (`Core/Src/main.c`)
- 蹂寃???
  - `osKernelStart()` ?꾨옒 `while (1)` ?대???怨쇨굅 踰좎뼱硫뷀깉 ?붿쟻(`app_loop()` 二쇱꽍, 誘몄궗??吏??蹂?????⑥븘 ?덉뿀??
  - RTOS 援ъ“?먯꽌 ?대떦 援ш컙? ?ㅼ젣濡??ㅽ뻾?섏? ?딅뒗??肄붾뱶媛 ?⑥븘 ?덉뼱 ?ㅽ빐 媛?μ꽦???믪븯??
- 蹂寃???
  - `while (1)` 釉붾줉 ?대?瑜?理쒖냼 ?뺥깭濡??뺣━.
  - 誘몄궗??蹂??怨쇨굅 ?몄텧 ?붿쟻 ?쒓굅.
  - 二쇱꽍??`RTOS: should not reach here`濡?紐낇솗???쒓린.
- 湲곗닠???④낵:
  - 湲곕뒫 蹂?붾뒗 ?놁쓬(?ㅽ뻾 寃쎈줈 ?숈씪).
  - ?좎?蹂댁닔 ??"???ш린 app_loop媛 ?놁??" 媛숈? ?쇰? 媛먯냼.

### B) HAL Timebase TIM11 遺꾨━ ?곸꽭
- 紐⑹쟻:
  - FreeRTOS ?ㅼ?以꾨쭅 tick(SysTick)怨?HAL tick(HAL_GetTick/HAL_Delay 湲곕컲)??遺꾨━?섏뿬 ?쒓컙異?媛꾩꽠 由ъ뒪?щ? ??땄.

#### 1) ?좉퇋 ?뚯씪 異붽?
- ?뚯씪: `Core/Src/stm32f4xx_hal_timebase_tim.c`
- ?듭떖 援ы쁽:
  - `HAL_InitTick(uint32_t TickPriority)`
    - APB2 ?대윮 湲곕컲?쇰줈 TIM11 prescaler 怨꾩궛.
    - TIM11??1MHz 移댁슫??1ms 二쇨린濡??ㅼ젙.
    - TIM11 update interrupt ?쒖옉.
  - `HAL_SuspendTick()` / `HAL_ResumeTick()`
    - TIM11 update IT on/off ?쒖뼱.
  - `HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)`
    - `htim->Instance == TIM11`???뚮쭔 `HAL_IncTick()` ?몄텧.

#### 2) ?명꽣?쏀듃 ?곌껐
- ?뚯씪: `Core/Src/stm32f4xx_it.c`, `Core/Inc/stm32f4xx_it.h`
- 蹂寃?
  - `TIM1_TRG_COM_TIM11_IRQHandler()` 異붽?.
  - ?몃뱾?ъ뿉??`HAL_TIM_IRQHandler(&htim11)` ?몄텧.
  - `SysTick_Handler()`?먯꽌 `HAL_IncTick()` ?쒓굅?섍퀬 RTOS tick 泥섎━(`xPortSysTickHandler`)留??좎?.

#### 3) MSP(HAL ?섏쐞 珥덇린?? ?곌껐
- ?뚯씪: `Core/Src/stm32f4xx_hal_msp.c`
- 蹂寃?
  - `HAL_TIM_Base_MspInit()`??`TIM11` 遺꾧린 異붽?:
    - `__HAL_RCC_TIM11_CLK_ENABLE()`
    - `HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TICK_INT_PRIORITY, 0)`
    - `HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn)`
  - `HAL_TIM_Base_MspDeInit()`??`TIM11` 遺꾧린 異붽?:
    - clock disable + IRQ disable

#### 4) IOC 諛섏쁺
- ?뚯씪: `irrled.ioc`
- 諛섏쁺媛?
  - `TIM11` IP 異붽?
  - `VP_TIM11_VS_ClockSourceINT` 異붽?
  - `TIM1_TRG_COM_TIM11_IRQn` ??ぉ 異붽?
  - TIM11 湲곕낯 二쇨린/遺꾩＜ ?뚮씪誘명꽣 諛섏쁺

### C) 蹂寃????쒓컙異??숈옉 ?붿빟
- SysTick:
  - FreeRTOS ?ㅼ?以꾨윭 tick ?꾩슜.
- TIM11 update interrupt:
  - HAL tick 利앷? ?꾩슜(`HAL_IncTick()`).
- 寃곌낵:
  - RTOS scheduling怨?HAL timebase媛 遺꾨━?섏뼱 援ъ“?곸쑝濡???紐낇솗?댁쭚.

### D) 二쇱쓽/?댁쁺 ?ъ씤??- CubeMX 肄붾뱶 ?ъ깮????
  - `stm32f4xx_hal_timebase_tim.c` ?좎? ?щ?
  - TIM11 IRQ/ MSP 遺꾧린 ?좎? ?щ?
  - SysTick ?몃뱾???댁슜(?섎룄移??딆? `HAL_IncTick()` ?ъ궫?? ?뺤씤 ?꾩슂.
- 濡쒖뺄 鍮뚮뱶:
  - ?꾩옱 ?묒뾽 ?섍꼍??`make`媛 ?놁뼱 鍮뚮뱶 ?ㅽ뻾? 誘몄닔??

---

## [v0.9.4] - 2026-03-05
### Dead Code ?뺣━(main.c)
- ????뚯씪: `Core/Src/main.c`
- 蹂寃??댁슜:
  - `osKernelStart()` ?댄썑??遺덊븘?뷀븳 `while` 釉붾줉 ?대? dead code ?뺣━
  - 怨쇨굅 `app_loop()` ?몄텧 ?붿쟻 諛?誘몄궗??吏??蹂???쒓굅
  - 二쇱꽍??`RTOS: should not reach here`濡?紐낇솗??- ?댁쑀:
  - ?ㅼ?以꾨윭 ?쒖옉 ???대떦 援ш컙???ㅽ뻾?섏? ?딆븘 ?쇰룞???좊컻?섎?濡?媛?낆꽦/?좎?蹂댁닔??媛쒖꽑

### HAL Timebase瑜?TIM11濡?遺꾨━
- ????뚯씪:
  - `Core/Src/stm32f4xx_hal_timebase_tim.c` (?좉퇋)
  - `Core/Src/stm32f4xx_it.c`
  - `Core/Inc/stm32f4xx_it.h`
  - `Core/Src/stm32f4xx_hal_msp.c`
  - `irrled.ioc`
- 蹂寃??댁슜(肄붾뱶):
  - ?좉퇋 `HAL_InitTick()` 援ы쁽:
    - `TIM11`??1ms 二쇨린濡??ㅼ젙/?쒖옉
    - `HAL_SuspendTick()`, `HAL_ResumeTick()` 援ы쁽
    - `HAL_TIM_PeriodElapsedCallback()`?먯꽌 `TIM11` update ?대깽????`HAL_IncTick()` ?몄텧
  - IRQ ?곌껐:
    - `TIM1_TRG_COM_TIM11_IRQHandler()` 異붽?
    - ?몃뱾???대? `HAL_TIM_IRQHandler(&htim11)` ?몄텧
  - `SysTick_Handler()`:
    - `HAL_IncTick()` ?쒓굅
    - FreeRTOS tick(`xPortSysTickHandler`) ?꾩슜?쇰줈 ?ъ슜
  - MSP ?ㅼ젙:
    - `TIM11` clock enable/disable 異붽?
    - `TIM1_TRG_COM_TIM11_IRQn` priority/NVIC enable/disable 異붽?
- 蹂寃??댁슜(ioc):
  - `TIM11` IP 諛?媛???(`VP_TIM11_VS_ClockSourceINT`) 異붽?
  - `TIM11` NVIC ??ぉ 異붽?
  - Timebase 遺꾨━ 援ъ꽦 諛섏쁺
- ?댁쑀:
  - FreeRTOS SysTick怨?HAL tick??遺꾨━???쒓컙湲곕컲 異⑸룎 媛?μ꽦????텛怨??ㅻТ ?덉젙???μ긽

---

## [v0.9.3] - 2026-03-05
### ControlTask / SensorTask ??븷 遺꾨━ ?꾨즺
- ????뚯씪: `Core/Inc/app.h`, `Core/Src/app.c`, `Core/Src/main.c`
- 蹂寃?紐⑹쟻:
  - ?쒖뼱 寃쎈줈? ?쇱꽌 寃쎈줈瑜?遺꾨━?댁꽌, ?쇱꽌 泥섎━ 吏?곗씠 紐⑦꽣 ?쒖뼱 二쇨린瑜?留됱? ?딅룄濡?媛쒖꽑
  - RTOS ??븷 遺꾨━ ?먯튃??留욊쾶 Task 梨낆엫??紐낇솗??
### 1) app 怨꾩링 ?⑥닔 遺꾨━
- `Core/Inc/app.h`
  - 異붽? API:
    - `void app_control_loop(void);`
    - `void app_sensor_loop(void);`
- `Core/Src/app.c`
  - `app_control_loop()` 異붽?:
    - `drain_control_queue()`濡??쒖뼱 紐낅졊 癒쇱? 諛섏쁺
    - AUTO 紐⑤뱶?먯꽌留?`mic_process()` + `motor_ctrl_process()` ?ㅽ뻾
    - 紐⑤뱶/留덉씠??紐⑦꽣 ?쒖뼱 ?꾨떞
  - `app_sensor_loop()` 異붽?:
    - `aht10_process(now)` ?ㅽ뻾
    - 二쇨린 濡쒓렇(ADC/諛⑺뼢/?⑥뒿??PAN/TILT) 異쒕젰
    - ?쇱꽌 媛깆떊/?곹깭 ?쒖떆 ?꾨떞
  - `app_loop()`???섏쐞?명솚 wrapper濡??좎?:
    - ?대??먯꽌 `app_sensor_loop()` + `app_control_loop()` ?쒖꽌 ?몄텧

### 2) ?쒖뼱 紐낅졊 諛섏쁺 吏???⑥씪??ControlTask ?꾨떞)
- `Core/Src/app.c`
  - ?좉퇋 ?⑥닔:
    - `push_control_command(uint8_t cmd)`:
      - `control_queue`???쒖뼱 紐낅졊 push
    - `drain_control_queue(void)`:
      - `control_queue`瑜?non-blocking?쇰줈 鍮꾩슦硫?紐낅졊 ?곸슜
    - `apply_control_command(uint8_t cmd)`:
      - 湲곗〈 `handle_uart_command` ??븷???泥?      - `o/f/w/a/s/d` 紐낅졊 ?ㅼ젣 諛섏쁺
  - 蹂寃쎌젏:
    - UART2 RX ISR: 吏곸젒 ?쒖뼱 ???`push_control_command()`留??섑뻾
    - UART1 `0x04` ?꾨젅???뚯떛 ?? 吏곸젒 ?쒖뼱 ???`push_control_command()` ?섑뻾
    - 利? 紐⑦꽣/紐⑤뱶 ?곹깭 蹂寃쎌? ControlTask?먯꽌留?諛쒖깮

### 3) Task 二쇨린 議곗젙
- `Core/Src/main.c`
  - `StartControlTask()`:
    - `app_control_loop();`
    - `osDelay(10);`  (10ms ?쒖뼱 二쇨린)
  - `StartSensorTask()`:
    - `app_sensor_loop();`
    - `osDelay(20);`  (20ms ?쇱꽌/?곹깭 二쇨린)

### 4) 理쒖쥌 ?ㅽ뻾 ?먮쫫
- UART ISR:
  - ???곸옱留??섑뻾?섍퀬 利됱떆 return
- UartRxTask:
  - `uart_rx_queue`?먯꽌 諛붿씠???섏떊 -> ?꾨젅??議곕┰/?뚯떛
  - `0x04` 紐낅졊? `control_queue`濡??꾨떖
  - `0x05`???곹깭 ?묐떟 泥섎━
- ControlTask(10ms):
  - `control_queue` 紐낅졊 諛섏쁺
  - AUTO ??留덉씠??諛⑺뼢 ?먮떒 + 紐⑦꽣 ?쒖뼱
- SensorTask(20ms):
  - AHT10 ?곹깭癒몄떊/二쇨린 媛깆떊 + 濡쒓렇 異쒕젰

---

## [v0.9.2] - 2026-03-05
### UartRxTask瑜??ㅼ젣 UART1 ?꾨젅???뚯꽌濡??쒖꽦??- ????뚯씪: `Core/Inc/app.h`, `Core/Src/app.c`, `Core/Src/main.c`
- 蹂寃?紐⑹쟻:
  - UART ?꾨줈?좎퐳 ?뚯떛??`app_loop` 寃쎈줈?먯꽌 遺꾨━?섍퀬 `UartRxTask` ?꾨떞?쇰줈 ?꾪솚
  - RTOS 援ъ“??留욌뒗 ??븷 遺꾨━(???뚮퉬/?뚯떛/?꾨젅??泥섎━)
- 蹂寃??곸꽭:
  - `Core/Inc/app.h`
    - `void app_on_uart1_byte(uint8_t b);` 怨듦컻 API 異붽?
  - `Core/Src/app.c`
    - `app_on_uart1_byte()` 異붽?
      - ?낅젰 諛붿씠?몃? `proto_rx_feed_byte()` ?곹깭癒몄떊???ъ엯
      - ?꾩꽦 ?꾨젅?꾩쓣 `proto_pop_frame()`濡?爰쇰궡 `process_protocol_frame()` 泥섎━
    - `app_loop()`?먯꽌 UART1 ???쒕젅???꾨젅??泥섎━ 肄붾뱶 ?쒓굅
      - UART1 ?뚯떛 梨낆엫??Task濡??꾩쟾???대룞
  - `Core/Src/main.c`
    - `StartUartRxTask()`瑜??ㅼ궗??濡쒖쭅?쇰줈 蹂寃?      - `osMessageQueueGet(uart_rx_queueHandle, ..., osWaitForever)`濡?諛붿씠???湲?      - ?섏떊 諛붿씠?몃? `app_on_uart1_byte(rx_byte)`濡??꾨떖
- 理쒖쥌 ?숈옉 ?먮쫫:
  - ISR(UART1): ?먯뿉 1諛붿씠???곸옱 ??利됱떆 由ы꽩
  - UartRxTask: ?먯뿉??諛붿씠???섏떊 -> ?꾨젅??議곕┰ -> `0x04/0x05` 泥섎━
  - ControlTask: `app_loop()`濡??쇱꽌/紐⑦꽣 二쇨린 ?쒖뼱 ?좎?

---

## [v0.9.1] - 2026-03-05
### UART1 ISR 理쒖냼??Queue 湲곕컲) ?곸슜
- ????뚯씪: `Core/Src/app.c`
- 蹂寃?紐⑹쟻:
  - UART RX ?명꽣?쏀듃?먯꽌 ?뚯떛 遺?댁쓣 ?쒓굅?섏뿬 ISR ?ㅽ뻾 ?쒓컙??理쒖냼??  - ?꾨젅???쒕∼/吏???꾪뿕 ?꾪솕
- 蹂寃??곸꽭:
  - `HAL_UART_RxCpltCallback()`??`USART1` 寃쎈줈?먯꽌
    - 湲곗〈: `proto_rx_feed_byte(rx_data_rpi)` 吏곸젒 ?몄텧
    - 蹂寃? `osMessageQueuePut(uart_rx_queueHandle, &rx_data_rpi, 0, 0)`濡?1諛붿씠?????곸옱 ??利됱떆 由ы꽩
  - ISR 諛??뚯떛 ?⑥닔 異붽?:
    - `proto_drain_uart_rx_queue()`
    - `osMessageQueueGet(..., timeout=0)`濡??먮? 鍮꾩슦硫?`proto_rx_feed_byte()` ?몄텧
  - `app_loop()` ?쒖옉遺??`proto_drain_uart_rx_queue()` ?몄텧 異붽?
    - ISR?먯꽌 ?곸옱??UART1 諛붿씠?몃? Task 而⑦뀓?ㅽ듃?먯꽌 ?꾨젅??議곕┰?섎룄濡?蹂寃?- 異붽? ?좎뼵:
  - `#include "cmsis_os2.h"`
  - `extern osMessageQueueId_t uart_rx_queueHandle;`
- 李멸퀬:
  - ?먭? ?꾩쭅 ?앹꽦?섏? ?딆? 珥덇린 ?쒖젏?먮뒗 UART1 諛붿씠?몃? 臾댁떆?섎룄濡?媛??泥섎━(`uart_rx_queueHandle != NULL`)

---

## [v0.9.0] - 2026-03-04
### UART1(RPi) 諛붿씠?덈━ ?꾨젅??+ JSON ?뚯떛/?묐떟 異붽?
- ????뚯씪: `Core/Src/app.c`
- 異붽? 留ㅽ겕濡??곸닔:
  - `PROTO_MAX_PAYLOAD` (理쒕? payload 192B)
- 異붽? ?곹깭癒몄떊:
  - `proto_rx_state_t`
  - ?곹깭: `PROTO_RX_WAIT_CMD -> PROTO_RX_WAIT_LEN_0..3 -> PROTO_RX_WAIT_PAYLOAD`
- 異붽? 踰꾪띁/?곹깭 蹂??
  - `s_proto_state`, `s_proto_cmd`, `s_proto_len`, `s_proto_idx`
  - `s_proto_rx_buf[]`
  - `s_frame_ready`, `s_frame_cmd`, `s_frame_len`, `s_frame_payload[]`
- 異붽? ?⑥닔:
  - `proto_rx_feed_byte(uint8_t b)`
    - UART1?먯꽌 1諛붿씠?몄뵫 ?낅젰諛쏆븘 ?꾨젅??議곕┰
    - 湲몄씠 ?꾨뱶 4諛붿씠?몃뒗 big-endian ?댁꽍
    - 湲몄씠 珥덇낵 ??reset
  - `proto_publish_frame(...)`
    - ISR ?뚯떛 寃곌낵瑜??⑥씪 ?щ’ ?꾨젅??踰꾪띁??寃뚯떆
  - `proto_pop_frame(...)`
    - 硫붿씤 濡쒖쭅?먯꽌 寃뚯떆???꾨젅??爰쇰깂(IRQ 蹂댄샇)
  - `process_protocol_frame(...)`
    - `0x05`(?곹깭?붿껌) / `0x04`(紐⑦꽣紐낅졊) 遺꾧린 泥섎━
  - `parse_motor_command(...)`
    - JSON 臾몄옄?댁뿉??`"motor"` ?ㅻ? ?먯깋
    - 媛??좏겙??異붿텧???대? ?⑤Ц 紐낅졊(`o/f/w/a/s/d`)?쇰줈 留ㅽ븨
    - 留ㅽ븨 ?덉슜媛? `auto/manual/on/off/o/f/w/a/s/d`
  - `proto_uart1_send_packet(...)`
    - `CMD(1) + LEN(4) + JSON` ?뺥깭濡?UART1 ?≪떊
  - `proto_send_status_packet()`
    - ?곹깭 ?묐떟 JSON ?앹꽦:
    - `{"tmp":..,"hum":..,"dir":"..","tilt":..}`
  - `proto_send_motor_ack(...)`
    - 紐⑦꽣 紐낅졊 ACK JSON ?앹꽦:
    - `{"ok":1/0,"mode":"auto/manual","cmd":"..."}`
- UART 肄쒕갚 蹂寃?
  - `HAL_UART_RxCpltCallback`?먯꽌
    - `USART2`: 湲곗〈 1諛붿씠???섎룞 紐낅졊 泥섎━ ?좎?
    - `USART1`: `proto_rx_feed_byte()` ?몄텧 ???명꽣?쏀듃 ?ы솢?깊솕
- 珥덇린??蹂寃?
  - `app_init()`?먯꽌 `HAL_UART_Receive_IT(&huart1, &rx_data_rpi, 1)` 異붽?

### AUTO/MANUAL 異쒕젰 ?뺣낫 ?뺤옣
- ????뚯씪: `Core/Src/app.c`
- 異붽? ?⑥닔:
  - `pwm_to_deg(uint16_t pwm, uint16_t min_pwm, uint16_t max_pwm)`
    - PWM 踰붿쐞瑜?0~180?꾨줈 ?좏삎 蹂??- 濡쒓렇 蹂寃?
  - AUTO 濡쒓렇??`PAN/TILT` 媛곷룄 異붽?
  - MANUAL 濡쒓렇 異붽?:
    - `MODE:MANUAL | PAN:xdeg TILT:ydeg | PAN_PWM:... TILT_PWM:...`

### FreeRTOS ?ㅽ뻾 寃쎈줈 蹂댁젙
- ????뚯씪: `Core/Src/main.c`
- `StartControlTask()`??`app_loop()` ?몄텧 異붽?
  - `app_loop(); osDelay(5);`
- ?섎룄:
  - `osKernelStart()` ?댄썑 ?ㅼ쭏 ?숈옉??Task 而⑦뀓?ㅽ듃?먯꽌 ?섑뻾?섎룄濡?蹂댁젙

---

## [v0.8.0] - 2026-03-04
### RTOS 移쒗솕 吏??泥섎━濡??꾪솚
- ????뚯씪: `Core/Src/motor.c`, `Core/Src/servo_calc.c`
- 蹂寃??댁쑀:
  - RTOS ?섍꼍?먯꽌 `HAL_Delay` 釉붾줈???곹뼢??以꾩씠湲??꾪븿
- 蹂寃??댁슜:
  - `cmsis_os2.h` ?ы븿
  - `delay_ms(uint32_t ms)` ?ы띁 異붽?
    - 而ㅻ꼸 ?ㅽ뻾 以? `osDelay(ms)`
    - 而ㅻ꼸 ?쒖옉 ?? `HAL_Delay(ms)`
  - 湲곗〈 `HAL_Delay(...)` ?몄텧??`delay_ms(...)`濡?移섑솚

---

## [v0.7.0] - 2026-03-03
### ?먮룞/?섎룞 紐⑤뱶 ?쒖뼱 泥닿퀎 ?낅뜲?댄듃
- ????뚯씪: `Core/Src/app.c`, `Core/Src/motor_ctrl.c`, `Core/Inc/motor_ctrl.h`
- 紐⑤뱶 紐낅졊:
  - `o/O` -> AUTO
  - `f/F` -> MANUAL
- ?섎룞 紐낅졊:
  - `W/S`(?명듃), `A/D`(??
- ?⑥닔 異붽?/?ъ슜:
  - `motor_ctrl_enter_auto()`
  - `motor_ctrl_enter_manual()`
  - `manual_move_pan(int step)`
  - `manual_move_tilt(int step)`
- AUTO ?ъ쭊???????명듃 ?쇳꽣 蹂듦? 濡쒖쭅 異붽?

### 2異??ы떥???쒖뼱 ?뺤젙
- ????뚯씪: `Core/Src/motor_ctrl.c`
- 梨꾨꼸:
  - PAN -> `TIM3_CH1`
  - TILT -> `TIM3_CH2`
- ?곹깭 蹂??
  - `current_pan_pwm`, `current_tilt_pwm`
- 諛⑹뼱 濡쒖쭅:
  - ?쒓퀎 PWM clamp 泥섎━

---

## [v0.6.0] - 2026-03-03
### 留덉씠??諛⑺뼢 ?먯젙 寃뚯씠???⑥닚??- ????뚯씪: `Core/Src/mic.c`
- ?섏궗寃곗젙 寃뚯씠?몃? ?듭떖 3媛쒕줈 ?⑥닚??
  - `SOUND_TH`
  - `SNR_TH_Q8`
  - `DIR_RATIO_TH_Q8`
- ?쒓굅??諛⑺뼢 ?뺤젙 議곌굔:
  - `DIFF_TH`, `SNR_DIFF_TH_Q8`, `PEAK_DIFF_TH` (?섏궗寃곗젙 寃쎈줈?먯꽌 ?쒖쇅)
- 紐⑹쟻:
  - ?쒕떇 ?ъ씤??異뺤냼
  - ?ㅽ깘/誘명깘 ?몃젅?대뱶?ㅽ봽瑜??꾩옣?먯꽌 鍮좊Ⅴ寃?留욎텛湲??쎄쾶 媛쒖꽑

---

## [v0.5.0] - 2026-03-03
### AHT10 ?꾪꽣留?媛뺥솕 (N=10 FIFO ?대룞?됯퇏)
- ????뚯씪: `Core/Src/aht10.c`
- 異붽? ?곸닔/蹂??
  - `AHT10_FILTER_WIN = 10`
  - `s_temp_hist[]`, `s_humi_hist[]`
  - `s_hist_idx`, `s_hist_count`, `s_temp_sum`, `s_humi_sum`
- 異붽? ?⑥닔:
  - `aht10_filter_push(...)`
- ?숈옉:
  - ???섑뵆 ?낅젰 ??媛???ㅻ옒???섑뵆 ?쒓굅 ???⑷퀎 媛깆떊(O(1))
  - ?⑥씪 ?섑뵆 異쒕젰 ???理쒓렐 N媛??됯퇏 異쒕젰

---

## [v0.4.0] - 2026-03-03
### AHT10 紐⑤뱢 遺꾨━ 諛??곹깭癒몄떊 ?곸슜
- ????뚯씪: `Core/Src/aht10.c`, `Core/Inc/aht10.h`
- 援ъ꽦:
  - `aht10_init()`, `aht10_process()`, `aht10_get_data()`
- ?듭떖 濡쒖쭅:
  - 二쇨린 痢≪젙(湲곕낯 2000ms)
  - 蹂???湲???80ms)
  - BUSY 鍮꾪듃 ?뺤씤 ???ъ떆??- 紐⑹쟻:
  - ?쇱꽌 ?먭?諛쒖뿴 ?꾪솕
  - 痢≪젙 ?꾨즺 ???쎄린 ?ㅻ쪟 諛⑹?

---

## [v0.3.0] - 2026-03-03
### 紐⑤뱢 遺꾨━ 由ы뙥?좊쭅
- `mic`, `motor_ctrl`, `aht10`, `ir_led` 紐⑤뱢 遺꾨━
- `app.c`瑜??곌퀎 ?ㅼ??ㅽ듃?덉씠??怨꾩링?쇰줈 ?뺣━

### IR LED 肄붾뱶 遺꾨━
- ????뚯씪: `Core/Src/ir_led.c`, `Core/Inc/ir_led.h`
- PB0/PB1/PB2 ?쒖뼱瑜?蹂꾨룄 紐⑤뱢濡?遺꾨━

---

## [v0.2.0] - 2026-03-03
### ?섎룞 議곗옉 湲곕뒫 ?꾩엯
- UART ?⑤Ц 紐낅졊 湲곕컲 ?섎룞 議곗옉 異붽?
- ???명듃 step ?쒖뼱 諛?紐⑤뱶 ?꾪솚 ?숈옉 ?뺣━

---

## [v0.1.0] - 2026-03-03
### 珥덇린 踰꾩쟾
- ???留덉씠??諛⑺뼢 媛먯? + ?쒕낫 援щ룞 湲곕낯 ?숈옉
- DMA 湲곕컲 ADC ?섑뵆 泥섎━ 寃쎈줈 援ъ꽦




