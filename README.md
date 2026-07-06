# Soldier-Wearable Comms, Actuation & Vision Platform

**Arya Sanchihar** (Roll No. S260036) · Dept. of Electronics & Communication Engineering, IIT Kanpur
SURGE Intern, Department of Electrical Engineering, IIT Kanpur · Mentor: **Prof. Ketan Rajawat**

This repository collects three independently developed subsystems for a soldier-wearable platform: infrastructure-free mesh video networking, sensorless ESC firmware with a staged DroneCAN bring-up path, and a lightweight optical-flow stabilization pipeline for constrained embedded hardware.

## Projects

| Folder | Subsystem | Summary |
|---|---|---|
| [`01-batman-adv-mesh-network`](./01-batman-adv-mesh-network) | **Comms** | BATMAN-Adv Layer-2 mesh + GStreamer video streaming across Raspberry Pi nodes, with zero-config Avahi discovery. |
| [`02-dronecan-esc-firmware`](./02-dronecan-esc-firmware) | **Actuation** | Sensorless BLDC ESC firmware (STM32F303RET6 + DRV8353S) with a staged PWM → DroneCAN command bus. |
| [`03-sad-optical-flow-eis`](./03-sad-optical-flow-eis) | **Vision** | Integer-only SAD block-matching optical flow with EIS crop-correction for a 64 KB RAM Cortex-M4. |

## Motivation

Battlefield communication cannot depend on centralized infrastructure that adversaries can target physically or through electronic warfare. A soldier-wearable platform instead needs infrastructure-independent networking, robust low-level actuation, and stabilized real-time vision — three complementary subsystems, developed and validated independently, presented together here.

## System Integration & Conclusion

Though developed and tested largely independently, the three subsystems form complementary layers of one soldier-wearable platform: mesh comms for infrastructure-free video/telemetry, ESC firmware for low-level actuation, and SAD-based EIS for stabilized vision. The consistent lesson across all three: decouple a subsystem's core logic from hardware or infrastructure that may be temporarily unavailable — Docker-simulated mesh nodes standing in for physical hardware, and a validated PWM path standing in for DroneCAN while CAN-capable test hardware was unavailable — so development stays productive without hardware in hand.

## Future Work

- End-to-end DroneCAN RawCommand validation for motor drive
- Multi-node mesh field testing under realistic RF interference
- Migrate ESC control from six-step to field-oriented control (FOC)
- 5–10 km soldier-node hops via directional / sub-GHz radios with multi-hop batman-adv relay

## Documentation

See [`docs/`](./docs) for the project poster and references.

## License

Add a license (e.g. MIT, Apache-2.0) appropriate for your institution's policy before making this repository public. IIT Kanpur may have its own IP/publication guidelines for SURGE program output — check with your mentor before pushing sensitive results.
