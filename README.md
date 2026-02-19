# PS5 Controlled Smart Car (ESP32)

An ESP32-based smart car controlled using a PS5 DualSense controller.

This project uses a non-blocking firmware architecture (no `delay()`), differential drive motor mixing, and a priority-based buzzer state machine.

Documentation:
https://lukefrost112.github.io/RobotCar/

---

## Demo

Add a photo of your build:

![Car Overview](YOUR_IMAGE_PATH_HERE)

Example:
![Car Overview](car_photo.jpg)

Add a YouTube demo video (recommended):

<iframe width="560" height="315"
src="https://www.youtube.com/embed/YOUR_VIDEO_ID"
frameborder="0" allowfullscreen>
</iframe>

---

## Features

- PS5 DualSense Bluetooth control (ESP32)
- Differential drive mixing (throttle + steering)
- Deadzone filtering for precise control
- Fully non-blocking firmware (no `delay()`)
- Priority-based buzzer finite state machine:
  - Connect/disconnect beep patterns
  - Boost mode (fast aggressive buzz)
  - Reverse warning tone (slow buzz)
  - Horn mode
- RGB visual feedback system
- 10kHz PWM motor control (8-bit resolution)
- Hardware schematic documented with KiCad


---

## Firmware Architecture

The firmware follows a layered, non-blocking design.

Each loop iteration performs:

1. Read controller input
2. Decide buzzer state (priority logic)
3. Update buzzer system (single-writer to BUZ pin)
4. Update RGB feedback (non-blocking blink logic)
5. Compute drive command (throttle + steering mixing)
6. Map motor outputs to H-bridge pins
7. Write PWM values to motors

All timing is handled using `millis()`.

No `delay()` is used anywhere in the system.

This ensures:
- Responsive controller input
- Stable Bluetooth connection
- Smooth motor control
- Deterministic feedback behavior


---

## Buzzer System

The buzzer is implemented as a priority-based finite state machine.

### States

- PATTERN  → Finite event-based beep patterns (connect/disconnect)
- BOOST    → Fast square-wave buzz while boost is held
- HORN     → Constant tone while horn button is pressed
- REVERSE  → Slow square-wave warning tone while reversing
- STANDBY  → Silence

### Priority Order

PATTERN > BOOST > HORN > REVERSE > STANDBY

This guarantees that important system events (like disconnect) cannot be overridden by user input.

### Design Principle

Only one function is allowed to write to the buzzer pin.

All buzzer behavior is selected through state logic and executed in a single update function.  
This prevents multi-writer conflicts and timing glitches.

### Timing Model

- Finite patterns use step counters and duration arrays.
- Continuous modes (BOOST / REVERSE) use square-wave toggling with `millis()` timing.
- State transitions reset timers to ensure predictable behavior.


---

## Motor Control

The car uses differential drive control.

### Mixing Formula

left  = throttle + steering  
right = throttle - steering  

This allows:
- Straight movement when steering = 0
- Turning while moving forward
- Pivot turns when throttle is near zero

### Deadzone Filtering

Small joystick noise is removed using configurable deadzones for:
- Trigger inputs
- Left stick X axis

This prevents unwanted drift or micro-movements.

### Output Mapping

Motor commands are clamped to ±255 and mapped to H-bridge direction pins.

PWM Configuration:
- Frequency: 10kHz
- Resolution: 8-bit (0–255 range)

This provides smooth motor response while remaining stable for the ESP32.


---

## Hardware Overview

The hardware is documented using a full KiCad schematic.

### Core Components

- ESP32 Dev Board
- H-Bridge Motor Driver
- Active 5V Buzzer
- BC547 NPN Transistor (buzzer driver)
- RGB LED
- Voltage Regulator

### Power Design

- ESP32 logic operates at 3.3V
- Buzzer operates at 5V via transistor driver
- Motor driver powered from main supply
- All modules share a common ground

Correct grounding is critical for:
- Stable Bluetooth connection
- Preventing random resets
- Clean motor operation

### Buzzer Driver Wiring (Important)

The active buzzer is powered using a BC547 transistor:

- GPIO → 1kΩ resistor → Base
- Base → GND via 10kΩ (optional but recommended)
- Emitter → GND
- Collector → Buzzer negative
- Buzzer positive → +5V

This ensures:
- Proper volume
- Safe GPIO current
- Reliable operation


---

## Build Instructions

1) Flash the firmware to the ESP32.

2) Set your controller MAC address in the code:

    'ps5.begin("XX:XX:XX:XX:XX:XX");'

3) Verify all modules share a common ground.

4) Confirm motor driver wiring matches the schematic.

5) Power the system and pair the PS5 controller.

---

## Documentation Site

Full project documentation:
https://lukefrost112.github.io/RobotCar/

Includes:
- System architecture
- Buzzer subsystem explanation
- Motor control breakdown
- RGB feedback logic
- Hardware schematic

---

## License

MIT License
