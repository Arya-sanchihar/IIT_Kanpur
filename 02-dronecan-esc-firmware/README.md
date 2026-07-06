# DroneCAN-Staged ESC Firmware

**Actuation subsystem** — Sensorless BLDC control on STM32F303RET6 + DRV8353S.

## Problem Statement

A compact actuator needs sensorless motor control with no Hall or encoder wiring, a standardized command bus (DroneCAN) for future flight-controller compatibility, onboard safety telemetry, and field-updatable firmware — all while Cortex-capable test hardware was not yet available during bring-up.

## Methodology

- **Platform**: STM32F303RET6 (72 MHz) + DRV8353S 3-phase gate driver over SPI
- **Control**: sensorless six-step trapezoidal commutation, zero-crossing detection, >34 kHz PWM (ARR = 2099)
- **Staged command bus**: PWM input → validated DroneCAN RawCommand via libcanard → pending hardware
- Ramped `apply_throttle()` (`RAMP_STEP = 3` counts/ms) with watchdog-driven safe-state cutoff on command timeout

### Firmware state flow

```
Read Command (PWM / validated / CAN pending)
        │
        ▼
Watchdog Interval Check
        │
        ▼
6-Step Commutation via BEMF Zero-Cross
        │
        ▼
Fault? → Safe State   |   OK → Report Telemetry
```

## Results

- Throttle @ `g_throttle` = 4095 (full-scale, 12-bit) → 100% commanded output
- PWM out @ `g_pwm_pulse_us` = 1500 µs, `g_pwm_signal_valid` = 1 → confirmed active and valid
- Timer capture @ `TIM4→CNT` = 2137, `TIM4→CCR1` = 1500 → matches expected 1500-count pulse width
- Switching @ >34 kHz, 6-channel complementary gate drive
- Safety @ Overcurrent/Undervoltage/ADC checks → SPI driver fault reporting (fault injection testing pending)

Input-capture end-to-end mapping confirmed correct on real hardware; full six-step commutation and safety control loop are functionally complete on the Nucleo-64 board.

## Novelty

The firmware decouples its command source from its safety-critical core — commutation, ramping, and watchdog logic are fully validated on real hardware using PWM as a stand-in, so switching to DroneCAN later requires no changes to the proven control loop.

## Repository Layout (suggested)

```
02-dronecan-esc-firmware/
├── README.md
├── Core/                # STM32CubeIDE project source (main.c, drivers)
├── Drivers/             # DRV8353S SPI driver, HAL config
├── DroneCAN/            # libcanard integration (staged, pending CAN hardware)
├── test/                # fault-injection & debugger capture logs
└── docs/                # commutation timing diagrams
```

> Populate the folders above with your actual STM32CubeIDE project and firmware source, then commit.
