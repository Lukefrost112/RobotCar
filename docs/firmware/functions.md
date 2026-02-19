# Function Reference

This page describes what each major function does and why it exists.

## Input

### readController(ControllerInput &c)
Reads controller state and writes it into the `ControllerInput` struct.
This isolates hardware reads from the rest of the logic.

## Drive

### applyDeadzone(int value, int deadzone)
Zeroes small values to remove controller noise.

### computeDriveCommand(const ControllerInput c, DriveCommand &d)
Converts raw inputs into:
- throttle (r2 - l2)
- steering (lx)
- mixed left/right outputs

Also clamps left/right outputs to safe range.

### mapMotor(int cmd, StopMode stop)
Converts a signed motor command into two PWM channel outputs
to control direction using an H-bridge.

### writeMotorOutputs(const MotorOut left, const MotorOut right)
Writes PWM values using ESP32 LEDC channels.

## Feedback (RGB)

### setRGB(int r, int g, int b)
Sets the RGB pins HIGH/LOW.

### startRGBBlink(int r, int g, int b, int intervalMs, int blinkCount)
Prepares a non-blocking blink pattern.

### updateRGB(unsigned long nowMs)
Advances the blink pattern based on `millis()` timing.

## Feedback (Buzzer)

### startBeepPattern(int beepCount)
Prepares a finite beep pattern (connect/disconnect).

### updateBuzzer(unsigned long nowMs)
Plays the finite beep pattern without blocking.

### decideBuzzerState(...)
Selects which buzzer mode should be active this loop
based on current conditions (pattern playing, buttons, reverse, etc.).

### updateBuzzerSystem(BuzzerState state)
The only function allowed to directly drive the BUZ pin.
Executes the current buzzer mode.
