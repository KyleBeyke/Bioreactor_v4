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
- Minimum: Every 10ms
- For 60Hz: Complete cycle every 16.67ms
- For 50Hz: Complete cycle every 20ms

### Timing Considerations
- CircuitPython's timing isn't as precise as Arduino
- Use the fastest main loop possible
- Avoid blocking operations in your code
- Consider using separate coro/tasks if using asyncio

### Power Limitations
- Inductive loads (motors) may require derating
- Maximum load depends on your dimmer module
- Typical modules: 200W-4000W depending on model
- Always check heat dissipation

### Zero-Crossing Detection
- The zero-cross pin should pulse at 2x mains frequency
- 60Hz mains = 120Hz zero-crossings
- 50Hz mains = 100Hz zero-crossings
- Pull-up resistor is enabled on the input pin

## Advanced Usage

### Multiple Synchronized Dimmers

```python
# Multiple dimmers sharing zero-cross signal
dimmer1 = rbddimmer.RBDDimmer(board.D5, board.D2)
dimmer2 = rbddimmer.RBDDimmer(board.D6, board.D2)  # Same zero-cross pin
dimmer3 = rbddimmer.RBDDimmer(board.D9, board.D2)

# Initialize all
for dimmer in [dimmer1, dimmer2, dimmer3]:
    dimmer.begin()

# Update all in main loop
while True:
    dimmer1.update()
    dimmer2.update()
    dimmer3.update()
    time.sleep(0.001)
```

### Non-Blocking Ramping

```python
# Start multiple ramps
dimmer1.set_power_ramp(100, 3000)  # 3 second fade up
dimmer2.set_power_ramp(0, 1500)    # 1.5 second fade down

# Continue other operations while ramping
while True:
    # Ramps happen automatically during update()
    dimmer1.update()
    dimmer2.update()

    # Do other tasks here
    check_sensors()
    update_display()

    time.sleep(0.001)
```

### Creating Light Effects

```python
import math

# Breathing effect
def breathing_effect(dimmer, speed=1.0):
    angle = 0
    while True:
        # Sine wave for smooth breathing
        power = int(50 + 45 * math.sin(angle))
        dimmer.set_power(power)

        angle += 0.05 * speed
        if angle > 2 * math.pi:
            angle = 0

        dimmer.update()
        time.sleep(0.01)

# Candle flicker effect
import random

def candle_effect(dimmer):
    base_power = 70
    while True:
        # Random flicker around base power
        flicker = random.randint(-15, 10)
        power = max(40, min(85, base_power + flicker))
        dimmer.set_power(power)

        # Variable update rate for realism
        dimmer.update()
        time.sleep(random.uniform(0.05, 0.15))
```

## Troubleshooting

### Dimmer Not Working
1. Check wiring connections
2. Verify zero-crossing signal with oscilloscope/logic analyzer
3. Ensure `update()` is called frequently enough
4. Check AC mains frequency setting (50/60Hz)

### Flickering
1. Increase `update()` call frequency
2. Check for stable zero-crossing signal
3. Reduce other blocking operations in code
4. Try interrupt-based version if supported

### Limited Dimming Range
1. Some loads have minimum power requirements
2. LED bulbs may not dim below 10-20%
3. Check if load is dimmable
4. Verify dimmer module specifications

### Board Compatibility Issues

Different CircuitPython boards have varying capabilities:

```python
# For SAMD21 boards (slower)
UPDATE_INTERVAL = 0.005  # 5ms

# For SAMD51 boards (faster)
UPDATE_INTERVAL = 0.001  # 1ms

# For RP2040 boards
UPDATE_INTERVAL = 0.001  # 1ms

# For ESP32-S2/S3
UPDATE_INTERVAL = 0.002  # 2ms
```

## Safety Considerations

⚠️ **MAINS VOLTAGE WARNING**

1. **Isolation**: Always use properly isolated dimmer modules
2. **Enclosure**: Mount in proper electrical enclosure
3. **Wiring**: Use appropriate gauge wire for load current
4. **Grounding**: Ensure proper earth/ground connections
5. **Testing**: Test with low-power loads first
6. **Certification**: Use only certified dimmer modules
7. **Local Codes**: Follow local electrical codes

## Performance Optimization

### Memory Usage
```python
# Pre-allocate for better performance
import array

# Use array instead of list for power ramping
power_steps = array.array('B', range(101))  # 0-100
```

### Timing Optimization
```python
# Use monotonic_ns for better precision
import time

class OptimizedDimmer(rbddimmer.RBDDimmer):
    def update(self):
        # Use nanosecond timing
        current_ns = time.monotonic_ns()
        current_us = current_ns // 1000

        # Rest of update logic...
```

### Multi-Core Usage (RP2040)
```python
# On RP2040, run dimmer on second core
import _thread

def dimmer_core():
    while True:
        dimmer.update()
        time.sleep(0.001)

# Start on second core
_thread.start_new_thread(dimmer_core, ())
```

## Example Projects

### 1. Sunrise Alarm Clock
Gradually increase brightness over 30 minutes to simulate sunrise.

### 2. Music Reactive Lighting
Use ADC to read audio levels and map to brightness.

### 3. Motion-Activated Dimming
PIR sensor triggers smooth fade-in/out.

### 4. Temperature-Controlled Fan
Map temperature readings to fan speed via dimmer.

### 5. Plant Growth Light Controller
Schedule different intensities throughout the day.

## Limitations

1. **Timing Precision**: CircuitPython timing is less precise than Arduino
2. **Interrupt Support**: Not all boards support hardware interrupts
3. **PWM Conflicts**: Dimmer uses software timing, not hardware PWM
4. **Processing Speed**: Requires fast enough processor for smooth operation
5. **Multi-tasking**: Heavy operations can affect dimming smoothness

## Contributing

To contribute improvements:

1. Test with your specific hardware
2. Document any board-specific requirements
3. Maintain compatibility with existing API
4. Add examples for new features
5. Update documentation

## License

This CircuitPython port maintains compatibility with the original RBDDimmer library API.
Released under GLPv3 License.

## Version History

- 1.0.0: Initial CircuitPython port
  - Basic dimming functionality
  - Ramping support
  - Multi-dimmer capability
  - 50/60Hz support

## Credits

- Original Arduino library: RobotDyn
- CircuitPython port: Kyle Beyke
- Zero-crossing detection algorithm adapted from RBDDimmer