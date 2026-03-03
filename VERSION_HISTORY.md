# Version History

Project version change log.
Date format: `YYYY-MM-DD`

## [v0.5.0] - 2026-03-03
- Added AHT10 `N=10` ring-buffer moving average filter (FIFO behavior)
- Changed output from single-sample value to averaged value

## [v0.4.0] - 2026-03-03
- Updated mode command mapping over UART
  - `o`/`O`: auto mode (on)
  - `f`/`F`: manual mode (off)
- Kept manual `W/A/S/D` pan-tilt step control
- Added center-reset on auto mode entry

## [v0.3.0] - 2026-03-03
- Extended motor control to 2-axis
  - `TIM3_CH1`: pan (left/right)
  - `TIM3_CH2`: tilt (up/down)
- Added manual APIs (`manual_move_pan`, `manual_move_tilt`)
- Wired USART2 RX interrupt path (`HAL_UART_RxCpltCallback`, `USART2_IRQHandler`)

## [v0.2.0] - 2026-03-03
- Added AHT10 module split (`Core/Src/aht10.c`, `Core/Inc/aht10.h`)
- Applied periodic measurement (default 2000 ms) and conversion wait (80 ms)
- Linked temperature/humidity print output to app log

## [v0.1.0] - 2026-03-03
- Organized module split for audio detection and motor control
  - `mic` module: DMA audio processing and direction decision
  - `motor_ctrl` module: motor state machine
- Split LED control into `ir_led` module

---

## Template
```md
## [vX.Y.Z] - YYYY-MM-DD
- Change 1
- Change 2
```

