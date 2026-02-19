# PS5 Controlled Smart Car

ESP32-based smart car controlled via a PS5 DualSense controller.

## What makes this build different
- **Non-blocking firmware**: no `delay()` anywhere; all timing uses `millis()`.
- **Layered architecture**: input → decision → execution → hardware outputs.
- **Deterministic buzzer system**: single-writer + priority-based state machine.

## Features
- Differential drive mixing (throttle + steering)
- Deadzone filtering for stick/triggers
- RGB feedback (non-blocking blink patterns)
- Buzzer feedback:
  - Connect/disconnect patterns
  - Boost “angry dog” buzz
  - Reverse warning buzz
  - Horn

## Quick Start
- Controller connects via `ps5Controller` library
- Motor outputs are driven using ESP32 LEDC PWM channels
- Feedback subsystems are updated every loop (no blocking)
