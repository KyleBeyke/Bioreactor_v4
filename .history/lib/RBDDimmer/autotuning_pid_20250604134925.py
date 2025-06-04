"""
Autotuning PID Temperature Controller for Raspberry Pi Pico with CircuitPython
Controls RBDDimmer heating element based on DS18B20 temperature sensor feedback

Hardware Setup:
- DS18B20 Temperature Sensor (1-Wire): Yellow=Data, Red=VCC(3.3V), Black=GND
- RBDDimmer: Connected via your existing port
- Crucible heating element connected to RBDDimmer

Features:
- Ziegler-Nichols autotuning method
- Real-time PID control
- Data logging capabilities
- Safety limits and monitoring
"""

import time
import board
import digitalio
import analogio
import busio
import microcontroller
from onewire import OneWire
from ds18x20 import DS18X20
import math

class DS18B20Sensor:
    """DS18B20 Temperature Sensor Interface"""

    def __init__(self, data_pin):
        self.ow = OneWire(data_pin)
        self.ds = DS18X20(self.ow)
        self.devices = self.ow.scan()
        if not self.devices:
            raise RuntimeError("No DS18B20 sensors found!")
        print(f"Found {len(self.devices)} DS18B20 sensor(s)")

    def read_temperature(self):
        """Read temperature in Celsius"""
        try:
            for device in self.devices:
                return self.ds.temperature(device)
        except Exception as e:
            print(f"Temperature read error: {e}")
            return None

class RBDDimmer:
    """
    CircuitPython port of RBDDimmer functionality
    Assumes your existing library implementation
    """

    def __init__(self, output_pin, zero_cross_pin=None):
        self.output_pin = digitalio.DigitalInOut(output_pin)
        self.output_pin.direction = digitalio.Direction.OUTPUT
        self.power_level = 0  # 0-100%
        self.state = False

        # Zero-cross detection (optional)
        if zero_cross_pin:
            self.zero_cross = digitalio.DigitalInOut(zero_cross_pin)
            self.zero_cross.direction = digitalio.Direction.INPUT
            self.zero_cross.pull = digitalio.Pull.UP
        else:
            self.zero_cross = None

    def begin(self, mode="NORMAL", state=True):
        """Initialize dimmer"""
        self.state = state
        self.set_power(0)  # Start at 0%

    def set_power(self, power):
        """Set power level (0-100%)"""
        self.power_level = max(0, min(100, power))
        # In a real implementation, this would control AC phase cutting
        # For now, we'll use PWM approximation
        if self.state:
            duty_cycle = int((self.power_level / 100.0) * 65535)
            # Implementation depends on your actual RBDDimmer port

    def get_power(self):
        """Get current power level"""
        return self.power_level

    def set_state(self, state):
        """Turn dimmer on/off"""
        self.state = state
        if not state:
            self.set_power(0)

class PIDController:
    """PID Controller with autotuning capability"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        # Internal variables
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_time = time.monotonic()

        # Output limits
        self.output_min = 0.0
        self.output_max = 100.0

        # Autotuning variables
        self.autotuning = False
        self.tuning_data = []
        self.tuning_start_time = 0
        self.ultimate_gain = 0
        self.ultimate_period = 0

    def compute(self, process_variable):
        """Compute PID output"""
        current_time = time.monotonic()
        dt = current_time - self.last_time

        if dt <= 0.0:
            return self.output_min

        # Calculate error
        error = self.setpoint - process_variable

        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral

        # Derivative term
        self.derivative = (error - self.previous_error) / dt
        derivative = self.kd * self.derivative

        # Calculate output
        output = proportional + integral + derivative

        # Clamp output to limits
        output = max(self.output_min, min(self.output_max, output))

        # Store values for next iteration
        self.previous_error = error
        self.last_time = current_time

        return output

    def set_tunings(self, kp, ki, kd):
        """Set PID tuning parameters"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_setpoint(self, setpoint):
        """Set target setpoint"""
        self.setpoint = setpoint

    def reset(self):
        """Reset PID internal state"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.monotonic()

class ZieglerNicholsAutotuner:
    """Ziegler-Nichols autotuning implementation"""

    def __init__(self, pid_controller, process_control_func):
        self.pid = pid_controller
        self.control_func = process_control_func
        self.tuning_complete = False

        # Autotuning parameters
        self.test_amplitude = 5.0  # Temperature oscillation amplitude
        self.oscillation_data = []
        self.peak_times = []
        self.peak_values = []

    def start_autotuning(self, setpoint, test_duration=1800):  # 30 minutes
        """Start Ziegler-Nichols autotuning process"""
        print("Starting Ziegler-Nichols autotuning...")
        print(f"Setpoint: {setpoint}°C, Duration: {test_duration}s")

        self.pid.set_setpoint(setpoint)
        self.pid.reset()
        self.tuning_complete = False
        self.oscillation_data = []
        self.peak_times = []
        self.peak_values = []

        start_time = time.monotonic()
        phase = 0  # 0=heating, 1=cooling
        last_switch_time = start_time

        # Initial parameters for relay control - adjusted for crucible heating
        relay_output_high = 80.0  # Higher output for crucible thermal mass
        relay_output_low = 20.0   # Lower baseline to maintain some heat

        while (time.monotonic() - start_time) < test_duration:
            current_temp = self.get_temperature()
            if current_temp is None:
                continue

            current_time = time.monotonic()
            elapsed = current_time - start_time

            # Relay feedback method for autotuning
            if phase == 0:  # Heating phase
                if current_temp >= (setpoint + self.test_amplitude/2):
                    phase = 1
                    last_switch_time = current_time
                    self.peak_times.append(elapsed)
                    self.peak_values.append(current_temp)
                output = relay_output_high
            else:  # Cooling phase
                if current_temp <= (setpoint - self.test_amplitude/2):
                    phase = 0
                    last_switch_time = current_time
                    self.peak_times.append(elapsed)
                    self.peak_values.append(current_temp)
                output = relay_output_low

            # Apply control output
            self.control_func(output)

            # Store data for analysis
            self.oscillation_data.append({
                'time': elapsed,
                'temperature': current_temp,
                'output': output,
                'setpoint': setpoint
            })

            # Print progress
            if len(self.oscillation_data) % 30 == 0:  # Every 30 samples
                print(f"Autotuning: {elapsed:.1f}s, Temp: {current_temp:.2f}°C, Out: {output:.1f}%")

            time.sleep(1.0)  # 1-second sampling

        # Analyze results and calculate PID parameters
        self.analyze_oscillation_data()

    def get_temperature(self):
        """Get current temperature - override in implementation"""
        return 25.0  # Placeholder

    def analyze_oscillation_data(self):
        """Analyze oscillation data to determine PID parameters"""
        if len(self.peak_times) < 4:
            print("Insufficient oscillation data for autotuning")
            return False

        # Calculate ultimate period (Tu)
        periods = []
        for i in range(1, len(self.peak_times) - 1):
            period = 2 * (self.peak_times[i+1] - self.peak_times[i-1])
            periods.append(period)

        if not periods:
            print("Could not determine oscillation period")
            return False

        ultimate_period = sum(periods) / len(periods)

        # Calculate ultimate gain (Ku) - simplified approach
        # In practice, this requires more sophisticated analysis
        amplitude = abs(max(self.peak_values) - min(self.peak_values))
        ultimate_gain = 4.0 * self.test_amplitude / (math.pi * amplitude)

        # Apply Ziegler-Nichols PID tuning rules
        kp = 0.6 * ultimate_gain
        ki = 2.0 * kp / ultimate_period
        kd = kp * ultimate_period / 8.0

        print(f"\nAutotuning Results:")
        print(f"Ultimate Gain (Ku): {ultimate_gain:.4f}")
        print(f"Ultimate Period (Tu): {ultimate_period:.2f}s")
        print(f"Calculated PID Parameters:")
        print(f"  Kp: {kp:.4f}")
        print(f"  Ki: {ki:.6f}")
        print(f"  Kd: {kd:.4f}")

        # Apply new tuning parameters
        self.pid.set_tunings(kp, ki, kd)
        self.tuning_complete = True

        return True

class TemperatureController:
    """Main temperature controller class"""

    def __init__(self, temp_sensor_pin, dimmer_output_pin, dimmer_zc_pin=None):
        # Initialize hardware
        self.temp_sensor = DS18B20Sensor(temp_sensor_pin)
        self.dimmer = RBDDimmer(dimmer_output_pin, dimmer_zc_pin)

        # Initialize PID controller
        self.pid = PIDController(kp=2.0, ki=0.1, kd=1.0)

        # Initialize autotuner
        self.autotuner = ZieglerNicholsAutotuner(self.pid, self.set_heater_power)
        self.autotuner.get_temperature = self.get_temperature

        # Safety parameters for crucible heating
        self.max_temp = 1200.0  # Maximum safe temperature for typical crucible
        self.min_temp = 10.0    # Minimum operating temperature
        self.max_heating_rate = 10.0  # Max °C/min heating rate to prevent thermal shock
        self.emergency_shutdown = False

        # Crucible-specific parameters
        self.thermal_mass_compensation = 1.5  # Account for crucible thermal mass
        self.last_temp = None
        self.last_temp_time = None

        # Data logging
        self.log_data = []
        self.log_interval = 5.0  # Log every 5 seconds
        self.last_log_time = 0

        # Initialize dimmer
        self.dimmer.begin("NORMAL", True)

    def get_temperature(self):
        """Get current temperature with error handling and crucible safety checks"""
        temp = self.temp_sensor.read_temperature()
        if temp is None:
            print("Temperature sensor error!")
            return None

        current_time = time.monotonic()

        # Safety check for maximum temperature
        if temp > self.max_temp:
            print(f"EMERGENCY: Temperature too high ({temp:.2f}°C)!")
            self.emergency_shutdown = True
            self.dimmer.set_power(0)

        # Check heating rate to prevent thermal shock in crucible
        if self.last_temp is not None and self.last_temp_time is not None:
            time_diff = current_time - self.last_temp_time
            if time_diff > 0:
                heating_rate = (temp - self.last_temp) / (time_diff / 60.0)  # °C/min
                if heating_rate > self.max_heating_rate:
                    print(f"WARNING: Heating rate too fast ({heating_rate:.1f}°C/min)!")
                    # Could implement rate limiting here

        self.last_temp = temp
        self.last_temp_time = current_time
        return temp

    def set_heater_power(self, power):
        """Set heater power with safety checks"""
        if self.emergency_shutdown:
            power = 0

        power = max(0, min(100, power))
        self.dimmer.set_power(power)

    def run_autotuning(self, target_temp, duration=1800):
        """Run autotuning process"""
        print(f"Starting autotuning for target temperature: {target_temp}°C")

        # Safety check
        if target_temp > self.max_temp or target_temp < self.min_temp:
            print(f"Target temperature {target_temp}°C is outside safe range!")
            return False

        self.autotuner.start_autotuning(target_temp, duration)
        return self.autotuner.tuning_complete

    def run_control_loop(self, target_temp, duration=None):
        """Run main PID control loop"""
        print(f"Starting PID control loop, target: {target_temp}°C")

        self.pid.set_setpoint(target_temp)
        self.pid.reset()

        start_time = time.monotonic()

        try:
            while True:
                current_time = time.monotonic()

                # Check duration limit
                if duration and (current_time - start_time) > duration:
                    break

                # Get current temperature
                current_temp = self.get_temperature()
                if current_temp is None:
                    time.sleep(1)
                    continue

                # Emergency shutdown check
                if self.emergency_shutdown:
                    print("Emergency shutdown activated!")
                    break

                # Calculate PID output
                pid_output = self.pid.compute(current_temp)

                # Apply control output
                self.set_heater_power(pid_output)

                # Data logging
                if (current_time - self.last_log_time) >= self.log_interval:
                    self.log_data.append({
                        'timestamp': current_time,
                        'elapsed': current_time - start_time,
                        'temperature': current_temp,
                        'setpoint': target_temp,
                        'output': pid_output,
                        'error': target_temp - current_temp
                    })

                    print(f"T:{current_temp:.2f}°C SP:{target_temp:.2f}°C "
                          f"Out:{pid_output:.1f}% Err:{target_temp-current_temp:.2f}°C")

                    self.last_log_time = current_time

                time.sleep(0.5)  # 500ms control loop

        except KeyboardInterrupt:
            print("\nControl loop stopped by user")
        finally:
            self.dimmer.set_power(0)
            print("Heater turned off")

    def save_log_data(self, filename="temperature_log.csv"):
        """Save logged data to CSV file"""
        try:
            with open(filename, 'w') as f:
                if self.log_data:
                    # Write header
                    f.write("timestamp,elapsed,temperature,setpoint,output,error\n")

                    # Write data
                    for entry in self.log_data:
                        f.write(f"{entry['timestamp']:.2f},{entry['elapsed']:.2f},"
                               f"{entry['temperature']:.2f},{entry['setpoint']:.2f},"
                               f"{entry['output']:.2f},{entry['error']:.2f}\n")

            print(f"Data saved to {filename}")
        except Exception as e:
            print(f"Error saving data: {e}")

# Main usage example for crucible heating
def main():
    """Main function demonstrating crucible heating control"""

    # Pin assignments for Raspberry Pi Pico
    TEMP_SENSOR_PIN = board.GP2    # DS18B20 data pin
    DIMMER_OUTPUT_PIN = board.GP3  # RBDDimmer control pin
    DIMMER_ZC_PIN = board.GP4      # Zero-cross detection (optional)

    try:
        # Initialize controller
        print("=== CRUCIBLE TEMPERATURE CONTROLLER ===")
        print("Initializing hardware...")
        controller = TemperatureController(
            TEMP_SENSOR_PIN,
            DIMMER_OUTPUT_PIN,
            DIMMER_ZC_PIN
        )

        # Target temperature for your application
        target_temperature = 800.0  # °C - adjust for your crucible material needs

        print(f"Target temperature: {target_temperature}°C")
        print("WARNING: Ensure crucible is rated for target temperature!")
        print("SAFETY: Keep temperature probe immersed in liquid!")

        # Check initial temperature
        initial_temp = controller.get_temperature()
        if initial_temp is None:
            print("Cannot read temperature sensor. Check connections.")
            return

        print(f"Initial temperature: {initial_temp:.2f}°C")

        # Safety confirmation
        if target_temperature > 1000:
            response = input(f"WARNING: High temperature ({target_temperature}°C). Continue? (y/N): ")
            if response.lower() != 'y':
                print("Operation cancelled for safety.")
                return

        # Option 1: Run autotuning first (recommended for first use)
        print("\n=== AUTOTUNING PHASE ===")
        print("Starting autotuning process...")
        print("This will take approximately 30 minutes.")
        print("The crucible will oscillate around the setpoint temperature.")

        if controller.run_autotuning(target_temperature, duration=1800):  # 30 minutes
            print("Autotuning completed successfully!")
            print("PID parameters have been automatically calculated.")
        else:
            print("Autotuning failed, using default parameters optimized for crucible heating")
            # Set conservative parameters for crucible thermal mass
            controller.pid.set_tunings(kp=1.5, ki=0.05, kd=2.0)

        # Option 2: Manual PID tuning (uncomment to skip autotuning)
        # print("Using manual PID parameters...")
        # controller.pid.set_tunings(kp=1.8, ki=0.08, kd=1.5)  # Conservative for crucible

        # Run control loop
        print("\n=== CONTROL PHASE ===")
        print("Starting temperature control...")
        print("Press Ctrl+C to stop safely")

        controller.run_control_loop(target_temperature, duration=7200)  # 2 hours

        # Save data
        controller.save_log_data(f"crucible_heating_{int(target_temperature)}C.csv")

    except KeyboardInterrupt:
        print("\n=== SAFE SHUTDOWN ===")
        print("User requested shutdown...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Always ensure heater is turned off
        try:
            if 'controller' in locals():
                controller.dimmer.set_power(0)
                print("Heater safely turned off.")
        except:
            pass

if __name__ == "__main__":
    main()