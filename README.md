# EdgeDevice STM32 (NUCLEO-F401RE)

STM32 firmware for:
- dual-mic direction detection (ADC + DMA)
- pan/tilt servo control (TIM3 CH1/CH2 PWM)
- AHT10 temperature/humidity sensing (I2C1)
- mode switching over UART (`auto` vs `manual`)

## Current Features

### 1) Audio Direction Detection
- Two ADC channels are sampled by DMA as interleaved stereo data.
- Signal path in `mic.c`:
  - baseline removal (`|raw - base|`)
  - short-window signal tracking
  - long-window noise tracking
  - SNR-based gating and direction ratio gate
- Main tuning constants:
  - `SOUND_TH`
  - `SNR_TH_Q8`
  - `DIR_RATIO_TH_Q8`
  - `SIG_WIN`, `NOISE_WIN`

### 2) Pan/Tilt Servo Control
- Pan: `TIM3_CH1` (PA6)
- Tilt: `TIM3_CH2` (PA7)
- PWM timing in current code:
  - Prescaler: `83`
  - Period: `19999`
  - 50 Hz servo-friendly frame

### 3) AHT10 Sensor
- I2C1 on:
  - SCL: PB8
  - SDA: PB9
- Measurement cycle: default `2000 ms`
- Conversion wait: `80 ms`
- Filtering: `N=10` ring-buffer moving average (FIFO behavior)

### 4) Mode Control (UART)
- UART2 command input in `app.c`:
  - `o` / `O`: auto mode (sound tracking enabled)
  - `f` / `F`: manual mode (sound tracking disabled)
  - manual mode movement:
    - `W/w`: tilt up
    - `S/s`: tilt down
    - `A/a`: pan left
    - `D/d`: pan right
- On auto mode entry, pan/tilt return to center.

### 5) LED Module Split
- LED control was split to `ir_led.c/.h`.
- Current boot behavior turns ON PB0/PB1/PB2.

## Runtime Behavior Summary

- `app_init()`:
  - initializes motor/mic/AHT10
  - starts UART RX interrupt (1-byte command handling)
- `app_loop()`:
  - always updates AHT10 state machine
  - in auto mode only:
    - runs `mic_process()`
    - runs `motor_ctrl_process()`
    - prints debug line every 200 ms
- In manual mode, audio debug print is disabled.

## Key Pin Map

- `PA6` -> `TIM3_CH1` (pan servo)
- `PA7` -> `TIM3_CH2` (tilt servo)
- `PB8` -> `I2C1_SCL` (AHT10)
- `PB9` -> `I2C1_SDA` (AHT10)
- `PA2` -> `USART2_TX` (terminal/debug)
- `PA3` -> `USART2_RX` (command input)
- `PA9` -> `USART1_TX` (reserved/available)
- `PA10` -> `USART1_RX` (reserved/available)
- `PB0/PB1/PB2` -> IR LED GPIO outputs

## Source Layout

- `Core/Src/app.c`: app orchestration, mode handling, UART command callback
- `Core/Src/mic.c`: ADC DMA audio processing and direction decision
- `Core/Src/motor_ctrl.c`: pan/tilt auto/manual motor logic
- `Core/Src/aht10.c`: AHT10 protocol + conversion timing + moving average filter
- `Core/Src/ir_led.c`: IR LED GPIO helpers

## Build Notes

- Project is generated from `irrled.ioc` (STM32CubeIDE project).
- If a newly added source file is not compiled, run:
  1. Project Refresh
  2. Clean
  3. Build

## Known Next Step

- JSON framed UART protocol (`0x04` / `0x05` + length + payload) is planned but not yet integrated.
- Current active command protocol is still single-byte UART commands (`o/f/WASD`).
