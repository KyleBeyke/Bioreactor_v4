# rbddimmer.py
"""
CircuitPython library for RobotDyn AC Dimmer
Converted from the Arduino RBDDimmer library

This library controls AC loads using phase control with zero-crossing detection.
Compatible with CircuitPython boards that support PWM and interrupts.

Example usage:
    import board
    import rbddimmer

    dimmer = rbddimmer.RBDDimmer(board.D5, board.D2)  # PWM pin, Zero-cross pin
    dimmer.begin()
    dimmer.set_power(50)  # Set to 50% power
"""

import time
import board
import digitalio
import pwmio
import countio
import array
import math

class RBDDimmer:
    """AC Dimmer control using phase angle control with zero-crossing detection"""

    # Constants
    DIMMER_NORMAL = 0
    DIMMER_RAMP = 1

    # Timing constants (in microseconds)
    PULSE_WIDTH = 100  # Triac trigger pulse width

    # Power to timing conversion table (for 50Hz and 60Hz)
    # Index is power level (0-100), value is delay in microseconds
    POWER_TABLE_50HZ = array.array('H', [
        10000, 9900, 9800, 9700, 9600, 9500, 9400, 9300, 9200, 9100,  # 0-9%
        9000, 8900, 8800, 8700, 8600, 8500, 8400, 8300, 8200, 8100,   # 10-19%
        8000, 7900, 7800, 7700, 7600, 7500, 7400, 7300, 7200, 7100,   # 20-29%
        7000, 6900, 6800, 6700, 6600, 6500, 6400, 6300, 6200, 6100,   # 30-39%
        6000, 5900, 5800, 5700, 5600, 5500, 5400, 5300, 5200, 5100,   # 40-49%
        5000, 4900, 4800, 4700, 4600, 4500, 4400, 4300, 4200, 4100,   # 50-59%
        4000, 3900, 3800, 3700, 3600, 3500, 3400, 3300, 3200, 3100,   # 60-69%
        3000, 2900, 2800, 2700, 2600, 2500, 2400, 2300, 2200, 2100,   # 70-79%
        2000, 1900, 1800, 1700, 1600, 1500, 1400, 1300, 1200, 1100,   # 80-89%
        1000, 900, 800, 700, 600, 500, 400, 300, 200, 100, 0          # 90-100%
    ])

    POWER_TABLE_60HZ = array.array('H', [
        8333, 8250, 8167, 8083, 8000, 7917, 7833, 7750, 7667, 7583,   # 0-9%
        7500, 7417, 7333, 7250, 7167, 7083, 7000, 6917, 6833, 6750,   # 10-19%
        6667, 6583, 6500, 6417, 6333, 6250, 6167, 6083, 6000, 5917,   # 20-29%
        5833, 5750, 5667, 5583, 5500, 5417, 5333, 5250, 5167, 5083,   # 30-39%
        5000, 4917, 4833, 4750, 4667, 4583, 4500, 4417, 4333, 4250,   # 40-49%
        4167, 4083, 4000, 3917, 3833, 3750, 3667, 3583, 3500, 3417,   # 50-59%
        3333, 3250, 3167, 3083, 3000, 2917, 2833, 2750, 2667, 2583,   # 60-69%
        2500, 2417, 2333, 2250, 2167, 2083, 2000, 1917, 1833, 1750,   # 70-79%
        1667, 1583, 1500, 1417, 1333, 1250, 1167, 1083, 1000, 917,    # 80-89%
        833, 750, 667, 583, 500, 417, 333, 250, 167, 83, 0            # 90-100%
    ])

    def __init__(self, output_pin, zero_cross_pin, frequency=60):
        """
        Initialize the dimmer

        Args:
            output_pin: Pin connected to dimmer module PWM input
            zero_cross_pin: Pin connected to zero-crossing detector
            frequency: AC mains frequency (50 or 60 Hz)
        """
        self.output_pin = output_pin
        self.zero_cross_pin = zero_cross_pin
        self.frequency = frequency

        # Select appropriate power table
        if frequency == 50:
            self.power_table = self.POWER_TABLE_50HZ
        else:
            self.power_table = self.POWER_TABLE_60HZ

        # State variables
        self.power_level = 0
        self.target_power = 0
        self.mode = self.DIMMER_NORMAL
        self._enabled = False
        self._ramp_counter = 0
        self._last_zero_cross = 0

        # Timing calculations
        self.half_cycle_us = 1000000 // (2 * frequency)  # Half cycle in microseconds

        # Initialize pins
        self.output = digitalio.DigitalInOut(output_pin)
        self.output.direction = digitalio.Direction.OUTPUT
        self.output.value = False

        # Zero-crossing input with pull-up
        self.zero_cross = digitalio.DigitalInOut(zero_cross_pin)
        self.zero_cross.direction = digitalio.Direction.INPUT
        self.zero_cross.pull = digitalio.Pull.UP

        # For edge detection
        self._last_zc_state = self.zero_cross.value

    def begin(self):
        """Initialize and start the dimmer"""
        self._enabled = True
        self.output.value = False

    def set_power(self, power):
        """
        Set dimmer power level immediately

        Args:
            power: Power level 0-100%
        """
        power = max(0, min(100, power))  # Clamp to 0-100
        self.power_level = power
        self.target_power = power
        self.mode = self.DIMMER_NORMAL

    def set_power_ramp(self, power, ramp_time_ms=1000):
        """
        Set dimmer power level with ramping

        Args:
            power: Target power level 0-100%
            ramp_time_ms: Time to ramp to target (milliseconds)
        """
        power = max(0, min(100, power))
        self.target_power = power
        self.mode = self.DIMMER_RAMP

        # Calculate ramp increment
        power_diff = abs(self.target_power - self.power_level)
        if power_diff > 0:
            self._ramp_step = 1 if self.target_power > self.power_level else -1
            self._ramp_delay_ms = ramp_time_ms / power_diff
            self._last_ramp_time = time.monotonic_ns() // 1_000_000
        else:
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
        Handles zero-crossing detection and phase control
        """
        if not self._enabled:
            return

        current_time_us = time.monotonic_ns() // 1000

        # Handle ramping
        if self.mode == self.DIMMER_RAMP:
            self._update_ramp()

        # Detect zero-crossing (falling edge)
        zc_state = self.zero_cross.value
        if self._last_zc_state and not zc_state:
            # Zero-crossing detected
            self._last_zero_cross = current_time_us
            self._last_zc_state = zc_state

            # Schedule triac firing if power > 0
            if self.power_level > 0 and self.power_level < 100:
                # Calculate delay from zero-cross to firing point
                delay_us = self.power_table[self.power_level]
                self._fire_time = self._last_zero_cross + delay_us
                self._fired = False
            elif self.power_level >= 100:
                # Full power - turn on immediately
                self.output.value = True
                self._fired = True
            else:
                # Zero power - keep off
                self.output.value = False
                self._fired = True
        else:
            self._last_zc_state = zc_state

        # Check if it's time to fire the triac
        if (hasattr(self, '_fire_time') and
            not getattr(self, '_fired', True) and
            current_time_us >= self._fire_time):

            # Fire the triac with a pulse
            self.output.value = True
            # Short delay for pulse width
            pulse_end = current_time_us + self.PULSE_WIDTH
            while time.monotonic_ns() // 1000 < pulse_end:
                pass
            self.output.value = False
            self._fired = True

    def _update_ramp(self):
        """Update ramping logic"""
        current_time_ms = time.monotonic_ns() // 1_000_000

        if current_time_ms - self._last_ramp_time >= self._ramp_delay_ms:
            self._last_ramp_time = current_time_ms

            if self.power_level != self.target_power:
                self.power_level += self._ramp_step

                # Check if we've reached target
                if ((self._ramp_step > 0 and self.power_level >= self.target_power) or
                    (self._ramp_step < 0 and self.power_level <= self.target_power)):
                    self.power_level = self.target_power
                    self.mode = self.DIMMER_NORMAL

    def change_frequency(self, frequency):
        """
        Change AC mains frequency

        Args:
            frequency: 50 or 60 Hz
        """
        self.frequency = frequency
        self.half_cycle_us = 1000000 // (2 * frequency)

        if frequency == 50:
            self.power_table = self.POWER_TABLE_50HZ
        else:
            self.power_table = self.POWER_TABLE_60HZ

    def off(self):
        """Turn off the dimmer"""
        self.set_power(0)
        self.output.value = False

    def deinit(self):
        """Cleanup when done"""
        self._enabled = False
        self.output.value = False
        self.output.deinit()
        self.zero_cross.deinit()


# Alternative implementation using interrupts (if supported by your board)
class RBDDimmerInterrupt(RBDDimmer):
    """
    Interrupt-based version for boards that support pin interrupts
    Note: Not all CircuitPython boards support interrupts
    """

    def __init__(self, output_pin, zero_cross_pin, frequency=60):
        super().__init__(output_pin, zero_cross_pin, frequency)
        self._interrupt_enabled = False

    def begin(self):
        """Initialize with interrupt support if available"""
        super().begin()

        # Try to set up interrupt (board-specific)
        try:
            # This is pseudo-code as CircuitPython interrupt support varies
            # You'll need to adapt this for your specific board
            # Some boards use countio, others use different methods
            self.counter = countio.Counter(self.zero_cross_pin, edge=countio.Edge.FALL)
            self._interrupt_enabled = True
        except:
            print("Interrupt not supported, using polling mode")
            self._interrupt_enabled = False

    def update(self):
        """Update function for interrupt mode"""
        if self._interrupt_enabled:
            # Check if we have new zero-crossings
            if self.counter.count > 0:
                self.counter.clear()
                self._handle_zero_cross()
        else:
            # Fall back to polling mode
            super().update()

    def _handle_zero_cross(self):
        """Handle zero-crossing event"""
        current_time_us = time.monotonic_ns() // 1000
        self._last_zero_cross = current_time_us

        # Schedule triac firing based on power level
        if self.power_level > 0 and self.power_level < 100:
            delay_us = self.power_table[self.power_level]
            # In a real implementation, you'd schedule a timer here
            # CircuitPython doesn't have great timer support, so we'll
            # need to check timing in the main loop
            self._fire_time = self._last_zero_cross + delay_us
            self._fired = False
        elif self.power_level >= 100:
            self.output.value = True
        else:
            self.output.value = False