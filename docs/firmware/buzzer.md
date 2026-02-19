# Buzzer System

## Goals
- Non-blocking sound (no delay)
- Clear priorities between sounds
- Predictable transitions between modes
- Single-writer control of `BUZ`

## Two categories of sound

### 1) Finite patterns (event-based)
Examples:
- controller connected
- controller disconnected

These have a clear start and end.

Implementation:
- `startBeepPattern()` prepares a short pattern (ON/OFF durations)
- `updateBuzzer()` advances the pattern with `millis()`

A pattern is considered “playing” when:
- `feedback.buzzerActive == true`

### 2) Continuous modes (condition-based)
Examples:
- BOOST is held → fast buzz
- reversing → slow buzz
- horn is held → constant ON

Implementation:
- `updateBuzzerSystem()` runs the current mode and toggles based on timers.

## BuzzerState Modes
- `PATTERN`: plays finite connect/disconnect pattern
- `BOOST`: fast square-wave toggle while boost held
- `REVERSE`: slow square-wave toggle while reversing
- `HORN`: constant HIGH while honk held
- `STANDBY`: forced LOW

## Priority (single active mode)
The firmware selects one mode each loop.
A typical priority order is:

PATTERN > HORN > BOOST > REVERSE > STANDBY

(You can swap BOOST/HORN if desired. The key is consistency.)

## Why state transitions reset timers
When switching modes, the system resets:
- the mode timer
- and often forces BUZ LOW

This ensures:
- no inherited timing from previous mode
- no mid-cycle half-beeps
- immediate stop when leaving a mode (especially BOOST)
