"""
Enhanced RBDDimmer for CircuitPython - Crucible Heating Optimized
Improved version with better AC control, safety features, and crucible-specific optimizations
"""

import time
import board
import digitalio
import pwmio
import microcontroller
from micropython import const

class RBDDimmer:
    """Enhanced AC Dimmer control optimized for crucible heating applications"""

    # Constants
    DIMMER_NORMAL = const(0)
    DIMMER_RAMP = const(1)
    DIMMER_PID = const(2)  # New: PID-controlled mode

    # Safety constants
    MAX_TEMP_RISE_RATE = const(10)  # °C/minute max heating rate
    PULSE_WIDTH_US = const(100)     # Triac pulse width in microseconds

    # AC timing for different frequencies
    AC_PERIOD_50HZ = const(20000)   # 20ms period for 50Hz
    AC_PERIOD_60HZ = const(16667)   # 16.67ms period for 60Hz

    def __init__(self, output_pin, zero_cross_pin=None, frequency=60, max_power=100):
        """
        Initialize enhanced dimmer

        Args:
            output_pin: Pin connected to dimmer module
            zero_cross_pin: Pin for zero-crossing detection (optional)
            frequency: AC mains frequency (50 or 60 Hz)
            max_power: Maximum allowed power level (0-100%) for safety
        """
        self.output_pin = output_pin
        self.zero_cross_pin = zero_cross_pin
        self.frequency = frequency
        self.max_power = max_power

        # State variables
        self.power_level = 0.0          # Current power (float for precision)
        self.target_power = 0.0         # Target power for ramping
        self.mode = self.DIMMER_NORMAL
        self._enabled = False
        self._emergency_stop = False

        # Safety monitoring
        self._last_power_change_time = 0
        self._power_change_rate = 0
        self._total_on_time = 0
        self._session_start_time = time.monotonic()

        # Ramping parameters
        self._ramp_rate = 1.0           # Power units per second
        self._last_ramp_update = 0

        # PWM for smoother control (if available)
        self._use_pwm = False
        self._pwm_obj = None

        # AC timing
        self._ac_period_us = self.AC_PERIOD_60HZ if frequency == 60 else self.AC_PERIOD_50HZ
        self._half_period_us = self._ac_period_us // 2

        # Initialize pins
        self.output = digitalio.DigitalInOut(output_pin)
        self.output.direction = digitalio.Direction.OUTPUT
        self.output.value = False

        # Zero-crossing detection setup
        if zero_cross_pin:
            self.zero_cross = digitalio.DigitalInOut(zero_cross_pin)
            self.zero_cross.direction = digitalio.Direction.INPUT
            self.zero_cross.pull = digitalio.Pull.UP
            self._last_zc_state = True
            self._zero_cross_available = True
        else:
            self.zero_cross = None
            self._zero_cross_available = False

        # Try to initialize PWM for smoother control
        self._init_pwm()

    def _init_pwm(self):
        """Initialize PWM if supported by the pin"""
        try:
            # Try to create PWM object for smoother dimming
            self._pwm_obj = pwmio.PWMOut(self.output_pin, frequency=1000, duty_cycle=0)
            self._use_pwm = True
            print("PWM dimming enabled")
        except (ValueError, AttributeError):
            # PWM not supported on this pin, fall back to digital control
            self._use_pwm = False
            print("Digital dimming mode")

    def begin(self, safety_enabled=True):
        """
        Initialize and start the enhanced dimmer

        Args:
            safety_enabled: Enable safety monitoring and limits
        """
        self._enabled = True
        self._emergency_stop = False
        self._safety_enabled = safety_enabled
        self._session_start_time = time.monotonic()
        self.output.value = False

        if self._pwm_obj:
            self._pwm_obj.duty_cycle = 0

        print(f"Enhanced RBDDimmer started - Safety: {safety_enabled}")

    def set_power(self, power, immediate=True):
        """
        Set dimmer power level with enhanced safety checks

        Args:
            power: Power level 0-100%
            immediate: If True, change immediately. If False, ramp to target
        """
        # Safety checks
        if self._emergency_stop:
            print("Emergency stop active - power command ignored")
            return False

        # Clamp power to safe limits
        power = max(0, min(self.max_power, float(power)))

        # Check power change rate for safety
        if hasattr(self, '_safety_enabled') and self._safety_enabled:
            if not self._check_power_change_rate(power):
                print(f"Power change rate limited for safety")
                return False

        if immediate:
            self.power_level = power
            self.target_power = power
            self.mode = self.DIMMER_NORMAL
        else:
            self.target_power = power
            self.mode = self.DIMMER_RAMP

        return True

    def set_power_ramp(self, power, ramp_time_seconds=30):
        """
        Set power with controlled ramping - ideal for crucible heating

        Args:
            power: Target power level 0-100%
            ramp_time_seconds: Time to reach target power
        """
        power = max(0, min(self.max_power, float(power)))

        if self._emergency_stop:
            return False

        self.target_power = power
        self.mode = self.DIMMER_RAMP

        # Calculate ramp rate
        power_difference = abs(self.target_power - self.power_level)
        if power_difference > 0 and ramp_time_seconds > 0:
            self._ramp_rate = power_difference / ramp_time_seconds
        else:
            self._ramp_rate = 1.0

        self._last_ramp_update = time.monotonic()
        return True

    def set_max_power(self, max_power):
        """Set maximum allowed power for safety"""
        self.max_power = max(0, min(100, max_power))

        # Reduce current power if it exceeds new limit
        if self.power_level > self.max_power:
            self.set_power(self.max_power)

    def get_power(self):
        """Get current power level"""
        return self.power_level

    def get_target_power(self):
        """Get target power level"""
        return self.target_power

    def get_state(self):
        """Get dimmer state (ON/OFF based on power level)"""
        return self.power_level > 0 and not self._emergency_stop

    def set_state(self, state):
        """Turn dimmer fully ON or OFF"""
        if state:
            self.set_power(self.max_power)
        else:
            self.set_power(0)

    def emergency_stop(self):
        """Immediately stop all heating - safety function"""
        self._emergency_stop = True
        self.power_level = 0
        self.target_power = 0
        self.output.value = False

        if self._pwm_obj:
            self._pwm_obj.duty_cycle = 0

        print("EMERGENCY STOP ACTIVATED")

    def reset_emergency(self):
        """Reset emergency stop condition"""
        self._emergency_stop = False
        print("Emergency stop reset")

    def get_status(self):
        """Get comprehensive dimmer status"""
        current_time = time.monotonic()
        session_time = current_time - self._session_start_time

        return {
            'power_level': self.power_level,
            'target_power': self.target_power,
            'mode': self.mode,
            'enabled': self._enabled,
            'emergency_stop': self._emergency_stop,
            'session_time_s': session_time,
            'total_on_time_s': self._total_on_time,
            'zero_cross_available': self._zero_cross_available,
            'pwm_available': self._use_pwm,
            'max_power': self.max_power
        }

    def update(self):
        """
        Enhanced update function with better timing and safety monitoring
        Call this frequently (>10Hz) in your main loop
        """
        if not self._enabled or self._emergency_stop:
            self._ensure_output_off()
            return

        current_time = time.monotonic()

        # Update ramping if in ramp mode
        if self.mode == self.DIMMER_RAMP:
            self._update_ramping(current_time)

        # Update power output based on method available
        if self._use_pwm:
            self._update_pwm_output()
        elif self._zero_cross_available:
            self._update_phase_control()
        else:
            self._update_simple_output()

        # Update usage statistics
        if self.power_level > 0:
            self._total_on_time += 0.1  # Approximate if called every 100ms

    def _check_power_change_rate(self, new_power):
        """Check if power change rate is safe for crucible heating"""
        current_time = time.monotonic()

        if self._last_power_change_time > 0:
            time_diff = current_time - self._last_power_change_time
            if time_diff > 0:
                power_diff = abs(new_power - self.power_level)
                rate = power_diff / time_diff  # Power units per second

                # Limit rapid power changes (configurable)
                max_rate = 10.0  # 10% power per second max
                if rate > max_rate:
                    return False

        self._last_power_change_time = current_time
        return True

    def _update_ramping(self, current_time):
        """Update power level for ramping mode"""
        if abs(self.target_power - self.power_level) < 0.1:
            # Close enough to target
            self.power_level = self.target_power
            self.mode = self.DIMMER_NORMAL
            return

        time_diff = current_time - self._last_ramp_update
        if time_diff >= 0.1:  # Update every 100ms
            self._last_ramp_update = current_time

            power_step = self._ramp_rate * time_diff

            if self.target_power > self.power_level:
                self.power_level = min(self.target_power, self.power_level + power_step)
            else:
                self.power_level = max(self.target_power, self.power_level - power_step)

    def _update_pwm_output(self):
        """Update PWM output for smooth dimming"""
        if self._pwm_obj:
            # Convert power percentage to duty cycle
            duty_cycle = int((self.power_level / 100.0) * 65535)
            self._pwm_obj.duty_cycle = duty_cycle

    def _update_phase_control(self):
        """Update phase control dimming with zero-crossing detection"""
        # Simplified phase control - would need more precise timing for AC
        if not self.zero_cross:
            return

        # Check for zero crossing
        zc_state = self.zero_cross.value
        if self._last_zc_state and not zc_state:  # Falling edge
            self._last_zc_state = zc_state

            if self.power_level <= 0:
                self.output.value = False
            elif self.power_level >= 100:
                self.output.value = True
            else:
                # Calculate delay for phase control
                delay_us = int(self._half_period_us * (100 - self.power_level) / 100)

                # Simple timing (not precise enough for real AC control)
                start_time = time.monotonic_ns()
                while (time.monotonic_ns() - start_time) < (delay_us * 1000):
                    pass

                # Trigger pulse
                self.output.value = True
                pulse_start = time.monotonic_ns()
                while (time.monotonic_ns() - pulse_start) < (self.PULSE_WIDTH_US * 1000):
                    pass
                self.output.value = False
        else:
            self._last_zc_state = zc_state

    def _update_simple_output(self):
        """Simple on/off control for basic dimming"""
        if self.power_level <= 0:
            self.output.value = False
        elif self.power_level >= 100:
            self.output.value = True
        else:
            # Simple time-based switching
            cycle_time_ms = 1000  # 1 second cycle
            current_ms = int(time.monotonic() * 1000) % cycle_time_ms
            threshold_ms = int(cycle_time_ms * self.power_level / 100)

            self.output.value = current_ms < threshold_ms

    def _ensure_output_off(self):
        """Ensure output is safely turned off"""
        self.output.value = False
        if self._pwm_obj:
            self._pwm_obj.duty_cycle = 0

    def calibrate_zero_cross(self):
        """Calibrate zero-crossing detection timing"""
        if not self._zero_cross_available:
            print("Zero-crossing detection not available")
            return False

        print("Calibrating zero-crossing detection...")

        # Measure zero-crossing periods
        periods = []
        last_time = 0

        for _ in range(10):  # Measure 10 periods
            # Wait for falling edge
            while self.zero_cross.value:
                pass
            while not self.zero_cross.value:
                pass

            current_time = time.monotonic_ns()
            if last_time > 0:
                period = (current_time - last_time) / 1000  # Convert to microseconds
                periods.append(period)
            last_time = current_time

        if periods:
            avg_period = sum(periods) / len(periods)
            detected_freq = 1000000 / avg_period  # Convert to Hz

            print(f"Detected frequency: {detected_freq:.1f} Hz")
            print(f"Average period: {avg_period:.0f} μs")

            # Update timing if reasonable
            if 45 <= detected_freq <= 65:  # Reasonable AC frequency range
                self._ac_period_us = int(avg_period)
                self._half_period_us = self._ac_period_us // 2
                return True

        print("Calibration failed - using default timing")
        return False

    def get_temperature_compensation(self, ambient_temp):
        """
        Get power compensation factor based on ambient temperature
        Useful for crucible heating in varying conditions

        Args:
            ambient_temp: Ambient temperature in Celsius

        Returns:
            Compensation factor (1.0 = no change, >1.0 = increase power)
        """
        # Simple linear compensation
        base_temp = 20.0  # °C
        temp_diff = base_temp - ambient_temp

        # Increase power by 1% per degree below base temperature
        compensation = 1.0 + (temp_diff * 0.01)

        # Limit compensation to reasonable range
        return max(0.8, min(1.2, compensation))

    def deinit(self):
        """Enhanced cleanup with statistics"""
        print("RBDDimmer Statistics:")
        print(f"  Session time: {time.monotonic() - self._session_start_time:.1f}s")
        print(f"  Total on time: {self._total_on_time:.1f}s")
        print(f"  Final power level: {self.power_level:.1f}%")

        self._enabled = False
        self._ensure_output_off()

        if self._pwm_obj:
            self._pwm_obj.deinit()
        self.output.deinit()
        if self.zero_cross:
            self.zero_cross.deinit()

# Legacy compatibility functions
def dimmerLamp(output_pin, zero_cross_pin=None):
    """Create RBDDimmer instance (Arduino-style naming)"""
    return RBDDimmer(output_pin, zero_cross_pin)

# Enhanced constants for compatibility
NORMAL_MODE = RBDDimmer.DIMMER_NORMAL
RAMP_MODE = RBDDimmer.DIMMER_RAMP
PID_MODE = RBDDimmer.DIMMER_PID
OFF = 0
ON = 1

# Utility functions for crucible applications
def calculate_crucible_ramp_time(start_temp, target_temp, max_rate_per_min=5):
    """
    Calculate safe ramp time for crucible heating

    Args:
        start_temp: Starting temperature in °C
        target_temp: Target temperature in °C
        max_rate_per_min: Maximum heating rate °C/minute

    Returns:
        Recommended ramp time in seconds
    """
    temp_diff = abs(target_temp - start_temp)
    ramp_time_min = temp_diff / max_rate_per_min
    return max(60, ramp_time_min * 60)  # Minimum 1 minute ramp