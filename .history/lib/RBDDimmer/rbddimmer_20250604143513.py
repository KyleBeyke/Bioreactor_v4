"""
Simplified RBDDimmer for CircuitPython - Compiles to .mpy
Removed problematic imports and simplified for Raspberry Pi Pico compatibility
"""

import time
import board
import digitalio

class RBDDimmer:
    """Simplified AC Dimmer control for CircuitPython"""

    # Constants
    DIMMER_NORMAL = 0
    DIMMER_RAMP = 1

    def __init__(self, output_pin, zero_cross_pin=None, frequency=60):
        """
        Initialize the dimmer
        Args:
            output_pin: Pin connected to dimmer module
            zero_cross_pin: Pin connected to zero-crossing detector (optional)
            frequency: AC mains frequency (50 or 60 Hz)
        """
        self.output_pin = output_pin
        self.zero_cross_pin = zero_cross_pin
        self.frequency = frequency

        # State variables
        self.power_level = 0
        self.target_power = 0
        self.mode = self.DIMMER_NORMAL
        self._enabled = False

        # Initialize output pin
        self.output = digitalio.DigitalInOut(output_pin)
        self.output.direction = digitalio.Direction.OUTPUT
        self.output.value = False

        # Initialize zero-crossing pin if provided
        if zero_cross_pin:
            self.zero_cross = digitalio.DigitalInOut(zero_cross_pin)
            self.zero_cross.direction = digitalio.Direction.INPUT
            self.zero_cross.pull = digitalio.Pull.UP
        else:
            self.zero_cross = None

    def begin(self):
        """Initialize and start the dimmer"""
        self._enabled = True
        self.output.value = False

    def set_power(self, power):
        """
        Set dimmer power level
        Args:
            power: Power level 0-100%
        """
        power = max(0, min(100, int(power)))
        self.power_level = power
        self.target_power = power
        self.mode = self.DIMMER_NORMAL

    def get_power(self):
        """Get current power level"""
        return self.power_level

    def get_state(self):
        """Get dimmer state (ON/OFF based on power level)"""
        return self.power_level > 0

    def set_state(self, state):
        """Turn dimmer fully ON or OFF"""
        self.set_power(100 if state else 0)

    def toggle_state(self):
        """Toggle between ON (100%) and OFF (0%)"""
        self.set_state(self.power_level == 0)

    def update(self):
        """
        Main update function - call this frequently in your main loop
        Simplified version without precise AC timing
        """
        if not self._enabled:
            return

        # Simple on/off control based on power level
        if self.power_level == 0:
            self.output.value = False
        elif self.power_level >= 100:
            self.output.value = True
        else:
            # For intermediate power levels, use simple PWM-like control
            # This is not true AC phase control but will work for basic dimming
            cycle_time = 0.02  # 20ms cycle (50Hz compatible)
            on_time = cycle_time * (self.power_level / 100.0)

            self.output.value = True
            time.sleep(on_time)
            self.output.value = False
            time.sleep(cycle_time - on_time)

    def off(self):
        """Turn off the dimmer"""
        self.set_power(0)
        self.output.value = False

    def deinit(self):
        """Cleanup when done"""
        self._enabled = False
        self.output.value = False
        self.output.deinit()
        if self.zero_cross:
            self.zero_cross.deinit()

# Legacy compatibility functions
def dimmerLamp(output_pin, zero_cross_pin=None):
    """Create RBDDimmer instance (Arduino-style naming)"""
    return RBDDimmer(output_pin, zero_cross_pin)

# Constants for compatibility
NORMAL_MODE = RBDDimmer.DIMMER_NORMAL
TOGGLE_MODE = RBDDimmer.DIMMER_RAMP
OFF = 0
ON = 1