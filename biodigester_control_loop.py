"""
CircuitPython-Compatible Biodigester Controller
Designed specifically for Raspberry Pi Pico with your hardware setup:
- DS18B20 temperature sensor
- SCD30 CO2 sensor
- RBDDimmer heating control
- Serial communication to Pi 4

Optimized for soil bacteria organic decomposition
"""

import time
import board
import digitalio
import busio
import json
from onewire import OneWire
from ds18x20 import DS18X20
import adafruit_scd30

class SimpleBiodigester:
    """Simplified biodigester controller for CircuitPython"""

    def __init__(self, temp_pin, dimmer_pin, i2c_sda, i2c_scl, uart_tx, uart_rx):
        """
        Initialize biodigester controller with your exact hardware setup

        Args:
            temp_pin: DS18B20 data pin (GP2)
            dimmer_pin: RBDDimmer control pin (GP3)
            i2c_sda: SCD30 I2C data pin (GP0)
            i2c_scl: SCD30 I2C clock pin (GP1)
            uart_tx: UART to Pi 4 TX pin (GP4)
            uart_rx: UART from Pi 4 RX pin (GP5)
        """

        # Temperature profiles - simplified for CircuitPython
        self.profiles = {
            'gentle': {'target': 35, 'max_power': 30, 'ramp_rate': 1.0},
            'optimal': {'target': 43, 'max_power': 40, 'ramp_rate': 0.5},
            'thermophilic': {'target': 55, 'max_power': 60, 'ramp_rate': 1.5}
        }

        # Set default profile
        self.current_profile = self.profiles['optimal']  # 43°C for soil bacteria

        # Initialize hardware
        self._init_temperature_sensor(temp_pin)
        self._init_co2_sensor(i2c_sda, i2c_scl)
        self._init_dimmer(dimmer_pin)
        self._init_uart(uart_tx, uart_rx)

        # Controller state
        self.target_temp = self.current_profile['target']
        self.current_temp = 20.0
        self.power_level = 0
        self.active = False
        self.emergency_stop = False

        # Timing
        self.last_update = 0
        self.last_log = 0
        self.session_start = time.monotonic()

        # Simple data storage (limited to avoid memory issues)
        self.temp_history = [20.0] * 10  # Last 10 readings
        self.co2_readings = [400] * 5    # Last 5 CO2 readings

        print("Biodigester controller initialized")
        print(f"Target temperature: {self.target_temp}°C")

    def _init_temperature_sensor(self, pin):
        """Initialize DS18B20 temperature sensor"""
        try:
            self.ow = OneWire(pin)
            self.ds18b20 = DS18X20(self.ow)
            self.temp_devices = self.ow.scan()
            if self.temp_devices:
                print("✓ DS18B20 temperature sensor ready")
            else:
                print("✗ No DS18B20 sensors found")
                self.temp_devices = None
        except Exception as e:
            print(f"✗ DS18B20 init error: {e}")
            self.temp_devices = None

    def _init_co2_sensor(self, sda_pin, scl_pin):
        """Initialize SCD30 CO2 sensor"""
        try:
            self.i2c = busio.I2C(scl_pin, sda_pin)
            self.scd30 = adafruit_scd30.SCD30(self.i2c)
            print("✓ SCD30 CO2 sensor ready")
        except Exception as e:
            print(f"✗ SCD30 init error: {e}")
            self.scd30 = None

    def _init_dimmer(self, pin):
        """Initialize simple dimmer control"""
        try:
            self.dimmer_pin = digitalio.DigitalInOut(pin)
            self.dimmer_pin.direction = digitalio.Direction.OUTPUT
            self.dimmer_pin.value = False
            print("✓ Dimmer control ready")
        except Exception as e:
            print(f"✗ Dimmer init error: {e}")
            self.dimmer_pin = None

    def _init_uart(self, tx_pin, rx_pin):
        """Initialize UART communication to Pi 4"""
        try:
            self.uart = busio.UART(tx_pin, rx_pin, baudrate=115200)
            print("✓ UART communication ready")
        except Exception as e:
            print(f"✗ UART init error: {e}")
            self.uart = None

    def read_temperature(self):
        """Read temperature from DS18B20"""
        if not self.temp_devices:
            return None

        try:
            for device in self.temp_devices:
                temp = self.ds18b20.temperature(device)
                if temp and 5 < temp < 80:  # Sanity check
                    return temp
        except Exception as e:
            print(f"Temperature read error: {e}")
        return None

    def read_co2(self):
        """Read CO2 from SCD30"""
        if not self.scd30:
            return None, None, None

        try:
            if self.scd30.data_available:
                co2 = self.scd30.CO2
                temp = self.scd30.temperature
                humidity = self.scd30.relative_humidity
                return co2, temp, humidity
        except Exception as e:
            print(f"CO2 read error: {e}")
        return None, None, None

    def set_profile(self, profile_name):
        """Change temperature profile"""
        if profile_name in self.profiles:
            self.current_profile = self.profiles[profile_name]
            self.target_temp = self.current_profile['target']
            print(f"Profile changed to {profile_name}: {self.target_temp}°C")
            return True
        return False

    def start_heating(self):
        """Start biodigester heating cycle"""
        self.active = True
        self.emergency_stop = False
        self.session_start = time.monotonic()
        print(f"Starting biodigester cycle - Target: {self.target_temp}°C")

    def stop_heating(self):
        """Stop heating safely"""
        self.active = False
        self.power_level = 0
        if self.dimmer_pin:
            self.dimmer_pin.value = False
        print("Heating stopped")

    def emergency_shutdown(self):
        """Emergency stop all heating"""
        self.emergency_stop = True
        self.active = False
        self.power_level = 0
        if self.dimmer_pin:
            self.dimmer_pin.value = False
        print("EMERGENCY STOP ACTIVATED")

    def calculate_power(self, temp_error):
        """Calculate heating power based on temperature error"""
        if not self.active or self.emergency_stop:
            return 0

        max_power = self.current_profile['max_power']

        # Very conservative power calculation for biological systems
        if abs(temp_error) <= 0.5:
            return 5  # Minimal maintenance heating
        elif abs(temp_error) <= 2.0:
            return min(15, max_power)  # Gentle heating
        elif abs(temp_error) <= 5.0:
            return min(25, max_power)  # Moderate heating
        else:
            return min(35, max_power)  # Maximum safe heating

    def update_heating(self):
        """Update heating based on current temperature"""
        if self.current_temp is None:
            return

        temp_error = self.target_temp - self.current_temp
        target_power = self.calculate_power(temp_error)

        # Smooth power changes to avoid thermal shock
        if abs(target_power - self.power_level) > 5:
            if target_power > self.power_level:
                self.power_level += 2  # Gentle increase
            else:
                self.power_level -= 3  # Faster decrease for safety
        else:
            self.power_level = target_power

        # Apply power using simple PWM-like control
        if self.dimmer_pin:
            if self.power_level > 50:
                self.dimmer_pin.value = True  # Full power
            elif self.power_level > 25:
                # Simple time-based switching for medium power
                cycle_time = int(time.monotonic() * 10) % 10
                self.dimmer_pin.value = cycle_time < 7
            elif self.power_level > 5:
                # Low power - shorter on time
                cycle_time = int(time.monotonic() * 10) % 10
                self.dimmer_pin.value = cycle_time < 3
            else:
                self.dimmer_pin.value = False

    def check_safety(self):
        """Simple safety checks"""
        if self.current_temp and self.current_temp > 50:
            print(f"Temperature too high: {self.current_temp}°C")
            self.emergency_shutdown()
            return False

        # Check for rapid temperature changes
        if len(self.temp_history) >= 3:
            recent_change = abs(self.temp_history[-1] - self.temp_history[-3])
            if recent_change > 3.0:  # More than 3°C change in 3 readings
                print(f"Temperature changing too rapidly: {recent_change}°C")
                self.emergency_shutdown()
                return False

        return True

    def update_history(self):
        """Update temperature history (simple circular buffer)"""
        if self.current_temp:
            self.temp_history.append(self.current_temp)
            if len(self.temp_history) > 10:
                self.temp_history.pop(0)

    def send_status_to_pi4(self):
        """Send status data to Raspberry Pi 4 via UART"""
        if not self.uart:
            return

        try:
            status = {
                'timestamp': time.monotonic(),
                'current_temp': self.current_temp,
                'target_temp': self.target_temp,
                'power_level': self.power_level,
                'active': self.active,
                'emergency_stop': self.emergency_stop,
                'co2_ppm': self.co2_readings[-1] if self.co2_readings else None,
                'session_time': time.monotonic() - self.session_start
            }

            json_str = json.dumps(status) + '\n'
            self.uart.write(json_str.encode())

        except Exception as e:
            print(f"UART send error: {e}")

    def process_commands(self):
        """Process commands from Pi 4"""
        if not self.uart or self.uart.in_waiting == 0:
            return

        try:
            data = self.uart.read(self.uart.in_waiting)
            if data:
                command_str = data.decode('utf-8').strip()
                if command_str:
                    command = json.loads(command_str)
                    self.handle_command(command)
        except Exception as e:
            print(f"Command processing error: {e}")

    def handle_command(self, command):
        """Handle commands from Pi 4"""
        cmd_type = command.get('command')

        if cmd_type == 'set_temperature':
            new_temp = command.get('value', self.target_temp)
            if 20 <= new_temp <= 60:
                self.target_temp = new_temp
                print(f"Target temperature set to {new_temp}°C")

        elif cmd_type == 'set_profile':
            profile = command.get('value', 'optimal')
            self.set_profile(profile)

        elif cmd_type == 'start_heating':
            self.start_heating()

        elif cmd_type == 'stop_heating':
            self.stop_heating()

        elif cmd_type == 'emergency_stop':
            self.emergency_shutdown()

    def print_status(self):
        """Print current status to console"""
        session_time = (time.monotonic() - self.session_start) / 3600
        co2_current = self.co2_readings[-1] if self.co2_readings else 0

        print(f"[{session_time:.1f}h] Temp: {self.current_temp:.1f}°C "
              f"(Target: {self.target_temp:.1f}°C) "
              f"Power: {self.power_level:.0f}% "
              f"CO2: {co2_current:.0f}ppm "
              f"Active: {self.active}")

    def run(self):
        """Main control loop - call this continuously"""
        current_time = time.monotonic()

        # Read sensors every 5 seconds
        if current_time - self.last_update >= 5.0:
            # Read temperature
            temp = self.read_temperature()
            if temp:
                self.current_temp = temp
                self.update_history()

            # Read CO2
            co2, _, _ = self.read_co2()
            if co2:
                self.co2_readings.append(co2)
                if len(self.co2_readings) > 5:
                    self.co2_readings.pop(0)

            self.last_update = current_time

        # Safety checks
        if not self.check_safety():
            return

        # Update heating control
        self.update_heating()

        # Process commands from Pi 4
        self.process_commands()

        # Send status every 10 seconds
        if current_time - self.last_log >= 10.0:
            self.send_status_to_pi4()
            self.print_status()
            self.last_log = current_time

# Main execution for your exact hardware setup
def main():
    """Main function using your specific pin assignments"""

    # Your pin assignments
    TEMP_PIN = board.GP2        # DS18B20 data
    DIMMER_PIN = board.GP3      # RBDDimmer control
    I2C_SDA = board.GP0         # SCD30 data
    I2C_SCL = board.GP1         # SCD30 clock
    UART_TX = board.GP4         # To Pi 4
    UART_RX = board.GP5         # From Pi 4

    print("Starting CircuitPython Biodigester Controller")
    print("Hardware: Pico + DS18B20 + SCD30 + RBDDimmer")
    print("=" * 50)

    try:
        # Initialize controller
        biodigester = SimpleBiodigester(
            TEMP_PIN, DIMMER_PIN,
            I2C_SDA, I2C_SCL,
            UART_TX, UART_RX
        )

        # Wait for initial sensor readings
        print("Waiting for initial sensor readings...")
        for _ in range(10):
            temp = biodigester.read_temperature()
            if temp:
                biodigester.current_temp = temp
                break
            time.sleep(1)

        print(f"Initial temperature: {biodigester.current_temp:.1f}°C")

        # Start heating cycle for soil bacteria
        biodigester.set_profile('optimal')  # 43°C target
        biodigester.start_heating()

        # Main control loop
        print("Starting biodigester control loop...")
        print("Send commands via UART from Pi 4")
        print("Press Ctrl+C to stop safely")

        while True:
            biodigester.run()
            time.sleep(0.5)  # 500ms loop time

    except KeyboardInterrupt:
        print("\nShutdown requested...")
        biodigester.stop_heating()
        print("Biodigester safely stopped")

    except Exception as e:
        print(f"Error: {e}")
        if 'biodigester' in locals():
            biodigester.emergency_shutdown()

if __name__ == "__main__":
    main()