# Architecture

## Main Loop Philosophy
The firmware is structured so that **all subsystems run concurrently** without blocking.

The loop performs a consistent pipeline:

1. Read controller inputs
2. Decide feedback states (buzzer/rgb)
3. Execute feedback updates (millis-based)
4. Compute drive command (throttle/steering mixing)
5. Map drive command to motor driver PWM pins
6. Write PWM to motors

## Why no delay()
Using `delay()` would freeze:
- controller responsiveness
- smooth motor control
- feedback timing accuracy

Instead, timing is implemented with:
- timestamps (`lastMs`)
- intervals (`intervalMs`)
- state variables (step counters / toggles)

## Subsystems
### Input
Reads controller state into a `ControllerInput` struct (snapshot).

### Drive
Converts input → `DriveCommand` → motor PWM outputs via:
- deadzones
- mixing
- clamping
- H-bridge mapping

### Feedback
Two independent non-blocking state machines:
- **RGB**: blink patterns driven by toggles and counters
- **Buzzer**: priority state machine that prevents multiple writers to BUZ pin
