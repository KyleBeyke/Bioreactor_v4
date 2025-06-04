# RBDDimmer CircuitPython Library

A CircuitPython library for controlling AC dimmers using phase control with zero-crossing detection. This is a port of the Arduino RBDDimmer library for use with CircuitPython-compatible boards.

## Features

- Phase angle control for smooth dimming
- Zero-crossing detection for flicker-free operation
- Support for 50Hz and 60Hz AC mains
- Smooth ramping/fading effects
- Multiple dimmer support
- Simple percentage-based power control (0-100%)

## Hardware Requirements

### Compatible Dimmer Modules
- RobotDyn AC Light Dimmer Module
- Similar triac-based dimmers with:
  - Zero-crossing detection output
  - Isolated trigger input (optocoupler)

### CircuitPython Board Requirements
- Digital output pin for dimmer control
- Digital input pin for zero-crossing detection
- Fast enough processor for timing (recommended: 48MHz+)
- CircuitPython 7.0 or newer

### Wiring

```
CircuitPython Board          Dimmer Module
-------------------          -------------
GND -----------------------> GND
Digital Output (D5) -------> PWM/Control Input
Digital Input (D2) --------> Zero-Cross Output

                            AC Load
                            -------
Dimmer Module              Light/Motor
-------------              -----------
AC-L ----------------------> Load Wire 1
AC-N ----------------------> Load Wire 2
AC Mains L ---------------> Live/Hot
AC Mains N ---------------> Neutral
```

⚠️ **WARNING**: This module works with dangerous mains voltage. Ensure proper isolation and safety measures. Never touch the AC side when powered.

## Installation

1. Copy `rbddimmer.py` to your CircuitPython device's `lib` folder
2. Import and use in your code:

```python
import board
import rbddimmer

dimmer = rbddimmer.RBDDimmer(board.D5, board.D2)
dimmer.begin()
dimmer.set_power(50)  # 50% brightness
```

## Basic Usage

### Simple Dimming

```python
import board
import time
import rbddimmer

# Create dimmer instance
# Parameters: output_pin, zero_cross_pin, frequency (50 or 60 Hz)
dimmer = rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60)

# Initialize
dimmer.begin()

# Main loop
while True:
    # Set to 75% power
    dimmer.set_power(75)

    # IMPORTANT: Must call update() frequently
    dimmer.update()
    time.sleep(0.001)  # 1ms delay
```

### Smooth Fading

```python
# Fade from 0% to 100% over 2 seconds
dimmer.set_power_ramp(100, 2000)

# Update until fade completes
while dimmer.get_power() != 100:
    dimmer.update()
    time.sleep(0.001)
```

## API Reference

### Constructor

```python
RBDDimmer(output_pin, zero_cross_pin, frequency=60)
```

- `output_pin`: Digital output pin connected to dimmer control input
- `zero_cross_pin`: Digital input pin connected to zero-crossing detector
- `frequency`: AC mains frequency (50 or 60 Hz)

### Methods

#### `begin()`
Initialize the dimmer. Must be called before use.

#### `set_power(power)`
Set dimmer power immediately.
- `power`: 0-100 (percentage)

#### `set_power_ramp(power, ramp_time_ms)`
Set power with smooth ramping.
- `power`: Target power (0-100)
- `ramp_time_ms`: Ramp duration in milliseconds

#### `get_power()`
Returns current power level (0-100).

#### `get_state()`
Returns `True` if dimmer is on (power > 0), `False` if off.

#### `set_state(state)`
Turn dimmer fully on (100%) or off (0%).
- `state`: `True` for on, `False` for off

#### `toggle_state()`
Toggle between on and off.

#### `update()`
Main update function. **Must be called frequently** (every 1-10ms) for proper operation.

#### `change_frequency(frequency)`
Change AC mains frequency.
- `frequency`: 50 or 60 (Hz)

#### `off()`
Turn off immediately.

#### `deinit()`
Clean up resources when done.

## Important Notes

### Update Frequency
The `update()` method must be called frequently for proper dimmer operation. Recommended rates:
- Ideal: Every 1ms