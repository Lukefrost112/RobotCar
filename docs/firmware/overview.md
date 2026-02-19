# Firmware Overview

## Key Data Structures

### ControllerInput
A snapshot of raw controller state:
- connection status
- steering axis (`lx`)
- triggers (`r2`, `l2`)
- action buttons (`honk`, `boost`)

### DriveCommand
Processed driving intent:
- `throttle`: forward/back value from triggers
- `steering`: steering value from stick
- `left`, `right`: mixed outputs for differential drive

### feedbackState
Tracks non-blocking timing + state for:
- buzzer pattern playback
- RGB blink playback

## Design Rule (important)
**Only one function should write to the buzzer pin.**

This prevents “tick” bugs caused by multiple parts of the code fighting over `BUZ`.
