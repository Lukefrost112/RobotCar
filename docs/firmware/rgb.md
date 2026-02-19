# RGB System

## Goal
Provide clear visual feedback without blocking motor control.

## How it works
The RGB subsystem uses:
- `rgbActive`: whether a blink pattern is running
- `rgbIntervalMs`: time between toggles
- `rgbBlinksLeft`: number of remaining ON pulses
- `rgbOn`: current on/off state

## startRGBBlink()
Prepares a pattern:
- saves target color
- sets blink count and interval
- resets timing state

## updateRGB()
Runs every loop:
- checks if interval elapsed
- toggles LED on/off
- decrements blink count only after completing an ON→OFF cycle
