# Hardware Design

This project includes a full KiCad schematic for accurate electrical documentation.

## Power Architecture

The system uses:
- 5V rail for buzzer and motor driver logic
- 3.3V logic for ESP32

All grounds are common.

## Buzzer Driver

The buzzer is driven using a BC547 NPN transistor:

- GPIO33 → 1kΩ resistor → Base
- Emitter → GND
- Collector → Buzzer negative
- Buzzer positive → 5V

This ensures:
- Proper volume
- No GPIO overcurrent
- Stable system operation

## Motor Driver

The motor driver is controlled via 4 PWM channels.

Differential drive logic is implemented in firmware.

![Full Schematic](assets/images/schematic.png)

![Power Section](assets/images/power_section.png)

![Buzzer Section](assets/images/buzzer_section.png)
