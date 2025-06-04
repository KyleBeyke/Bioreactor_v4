"""
Integrated CO2 and Temperature Monitoring System for Raspberry Pi Pico
Combines SCD30 CO2 sensor with temperature control and Pi 4 communication

Hardware:
- DS18B20 Temperature Sensor (liquid temperature)
- SCD30 CO2/Temperature/Humidity Sensor
- RBDDimmer for crucible heating control
- UART communication to Raspberry Pi 4

Features:
- Real-time CO2 and temperature monitoring
- PID temperature control with autotuning
- Serial communication protocol for Pi 4 control
- Data logging and safety monitoring
"""

import time
import board
import digitalio
import busio
import microcontroller
import json
from onewire import OneWire
from ds18x20 import DS18X20
import adafruit_scd30
import math

class SCD30Sensor:
    """SCD30 CO2, Temperature, and Humidity Sensor Interface"""

    def __init__(self, i2c_bus):
        try:
            self.scd30 = adafruit_scd30.SCD30(i2c_bus)
            print("SCD30 sensor initialized")

            # Configure sensor settings
            self.scd30.measurement_interval = 2  # 2 seconds
            self.scd30.self_calibration_enabled = True
            self.scd30.ambient_pressure = 1013  # mbar at sea level

            # Start continuous measurement
            print(f"SCD30 firmware version: {self.scd30.firmware_version}")

        except Exception as e:
            print(f"SCD30 initialization error: {e}")
            self.scd30 = None

    def read_data(self):
        """Read CO2, temperature, and humidity"""
        if self.scd30 is None:
            return None, None, None

        try:
            if self.scd30.data_available:
                co2 = self.scd30.CO2
                temperature = self.scd30.temperature
                humidity = self.scd30.relative_humidity
                return co2, temperature, humidity
            else:
                return None, None, None
        except Exception as e:
            print(f"SCD30 read error: {e}")
            return None, None, None

    def set_altitude_compensation(self, altitude_m):
        """Set altitude compensation in meters"""
        if self.scd30:
            self.scd30.altitude = altitude_m

    def force_calibration(self, co2_ppm=400):
        """Force calibration to known CO2 level (outdoor air ~400ppm)"""
        if self.scd30:
            self.scd30.forced_recalibration_reference = co2_ppm
            print(f"SCD30 calibrated to {co2_ppm} ppm")

class DS18B20Sensor:
    """DS18B20 Temperature Sensor Interface (for liquid temperature)"""

    def __init__(self, data_pin):
        try:
            self.ow = OneWire(data_pin)
            self.ds = DS18X20(self.ow)
            self.devices = self.ow.scan()
            if not self.devices:
                raise RuntimeError("No DS18B20 sensors found!")
            print(f"Found {len(self.devices)} DS18B20 sensor(s)")
        except Exception as e:
            print(f"DS18B20 initialization error: {e}")
            self.ds = None
            self.devices = []

    def read_temperature(self):
        """Read liquid temperature in Celsius"""
        if not self.ds or not self.devices:
            return None

        try:
            for device in self.devices:
                return self.ds.temperature(device)
        except Exception as e:
            print(f"DS18B20 read error: {e}")
            return None

class RBDDimmer:
    """RBDDimmer interface for heating element control"""

    def __init__(self, output_pin, zero_cross_pin=None):
        self.output_pin = digitalio.DigitalInOut(output_pin)
        self.output_pin.direction = digitalio.Direction.OUTPUT
        self.power_level = 0  # 0-100%
        self.state = False

        if zero_cross_pin:
            self.zero_cross = digitalio.DigitalInOut(zero_cross_pin)
            self.zero_cross.direction = digitalio.Direction.INPUT
            self.zero_cross.pull = digitalio.Pull.UP
        else:
            self.zero_cross = None

    def begin(self, mode="NORMAL", state=True):
        self.state = state
        self.set_power(0)

    def set_power(self, power):
        self.power_level = max(0, min(100, power))
        # Your RBDDimmer implementation here

    def get_power(self):
        return self.power_level

    def set_state(self, state):
        self.state = state
        if not state:
            self.set_power(0)

class PIDController:
    """PID Controller for temperature control"""

    def __init__(self, kp=1.5, ki=0.05, kd=2.0, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_time = time.monotonic()

        self.output_min = 0.0
        self.output_max = 100.0

    def compute(self, process_variable):
        current_time = time.monotonic()
        dt = current_time - self.last_time

        if dt <= 0.0:
            return self.output_min

        error = self.setpoint - process_variable

        proportional = self.kp * error
        self.integral += error * dt
        integral = self.ki * self.integral
        self.derivative = (error - self.previous_error) / dt
        derivative = self.kd * self.derivative

        output = proportional + integral + derivative
        output = max(self.output_min, min(self.output_max, output))

        self.previous_error = error
        self.last_time = current_time

        return output

    def set_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.monotonic()

class SerialProtocol:
    """Communication protocol for Raspberry Pi 4"""

    def __init__(self, uart):
        self.uart = uart
        self.command_buffer = ""

    def send_data(self, data_dict):
        """Send data as JSON to Pi 4"""
        try:
            json_str = json.dumps(data_dict) + "\n"
            self.uart.write(json_str.encode())
        except Exception as e:
            print(f"Serial send error: {e}")

    def check_commands(self):
        """Check for incoming commands from Pi 4"""
        commands = []

        try:
            if self.uart.in_waiting > 0:
                data = self.uart.read(self.uart.in_waiting)
                if data:
                    self.command_buffer += data.decode('utf-8', errors='ignore')

                    # Process complete lines
                    while '\n' in self.command_buffer:
                        line, self.command_buffer = self.command_buffer.split('\n', 1)
                        line = line.strip()

                        if line:
                            try:
                                command = json.loads(line)
                                commands.append(command)
                            except json.JSONDecodeError:
                                print(f"Invalid JSON command: {line}")

        except Exception as e:
            print(f"Serial receive error: {e}")

        return commands

class IntegratedMonitoringSystem:
    """Main system class integrating all sensors and control"""

    def __init__(self, temp_pin, dimmer_pin, dimmer_zc_pin=None,
                 i2c_sda=board.GP0, i2c_scl=board.GP1,
                 uart_tx=board.GP4, uart_rx=board.GP5):

        # Initialize I2C for SCD30
        self.i2c = busio.I2C(i2c_scl, i2c_sda)

        # Initialize UART for Pi 4 communication
        self.uart = busio.UART(uart_tx, uart_rx, baudrate=115200)

        # Initialize sensors
        self.temp_sensor = DS18B20Sensor(temp_pin)
        self.co2_sensor = SCD30Sensor(self.i2c)
        self.dimmer = RBDDimmer(dimmer_pin, dimmer_zc_pin)

        # Initialize controllers
        self.pid = PIDController(kp=1.5, ki=0.05, kd=2.0)
        self.protocol = SerialProtocol(self.uart)

        # System state
        self.control_enabled = False
        self.target_temperature = 25.0
        self.max_temp = 1200.0
        self.max_co2 = 10000  # ppm safety limit
        self.emergency_shutdown = False

        # Data logging
        self.log_interval = 5.0  # seconds
        self.last_log_time = 0
        self.data_history = []

        # Initialize hardware
        self.dimmer.begin("NORMAL", True)

        print("Integrated Monitoring System initialized")

    def read_all_sensors(self):
        """Read all sensor data"""
        # Read liquid temperature
        liquid_temp = self.temp_sensor.read_temperature()

        # Read CO2 sensor data
        co2_ppm, ambient_temp, humidity = self.co2_sensor.read_data()

        return {
            'timestamp': time.monotonic(),
            'liquid_temperature': liquid_temp,
            'ambient_temperature': ambient_temp,
            'co2_ppm': co2_ppm,
            'humidity': humidity,
            'heater_power': self.dimmer.get_power(),
            'target_temperature': self.target_temperature,
            'control_enabled': self.control_enabled
        }

    def process_commands(self):
        """Process commands from Raspberry Pi 4"""
        commands = self.protocol.check_commands()

        for cmd in commands:
            cmd_type = cmd.get('command')

            if cmd_type == 'set_temperature':
                new_temp = cmd.get('value', 25.0)
                if 10.0 <= new_temp <= self.max_temp:
                    self.target_temperature = new_temp
                    self.pid.set_setpoint(new_temp)
                    print(f"Temperature setpoint changed to {new_temp}°C")
                else:
                    print(f"Temperature {new_temp}°C out of range")

            elif cmd_type == 'enable_control':
                self.control_enabled = cmd.get('value', False)
                if not self.control_enabled:
                    self.dimmer.set_power(0)
                print(f"Temperature control {'enabled' if self.control_enabled else 'disabled'}")

            elif cmd_type == 'set_pid':
                kp = cmd.get('kp', self.pid.kp)
                ki = cmd.get('ki', self.pid.ki)
                kd = cmd.get('kd', self.pid.kd)
                self.pid.set_tunings(kp, ki, kd)
                print(f"PID parameters updated: Kp={kp}, Ki={ki}, Kd={kd}")

            elif cmd_type == 'emergency_stop':
                self.emergency_shutdown = True
                self.control_enabled = False
                self.dimmer.set_power(0)
                print("EMERGENCY STOP activated")

            elif cmd_type == 'reset_emergency':
                self.emergency_shutdown = False
                print("Emergency reset")

            elif cmd_type == 'calibrate_co2':
                ppm = cmd.get('value', 400)
                self.co2_sensor.force_calibration(ppm)

            elif cmd_type == 'get_status':
                # Send current status
                pass  # Status is sent regularly anyway

    def safety_checks(self, sensor_data):
        """Perform safety checks on sensor data"""
        liquid_temp = sensor_data.get('liquid_temperature')
        co2_ppm = sensor_data.get('co2_ppm')

        # Temperature safety
        if liquid_temp and liquid_temp > self.max_temp:
            print(f"EMERGENCY: Temperature too high ({liquid_temp:.1f}°C)")
            self.emergency_shutdown = True

        # CO2 safety
        if co2_ppm and co2_ppm > self.max_co2:
            print(f"WARNING: High CO2 level ({co2_ppm:.0f} ppm)")

        # Emergency shutdown
        if self.emergency_shutdown:
            self.control_enabled = False
            self.dimmer.set_power(0)

    def run_control_loop(self, sensor_data):
        """Run PID control loop"""
        if not self.control_enabled or self.emergency_shutdown:
            self.dimmer.set_power(0)
            return

        liquid_temp = sensor_data.get('liquid_temperature')
        if liquid_temp is None:
            print("Cannot control: No temperature reading")
            self.dimmer.set_power(0)
            return

        # Calculate PID output
        pid_output = self.pid.compute(liquid_temp)

        # Apply control output
        self.dimmer.set_power(pid_output)

    def send_data_to_pi4(self, sensor_data):
        """Send sensor data to Raspberry Pi 4"""
        # Add system status
        status_data = sensor_data.copy()
        status_data.update({
            'emergency_shutdown': self.emergency_shutdown,
            'pid_kp': self.pid.kp,
            'pid_ki': self.pid.ki,
            'pid_kd': self.pid.kd,
            'system_time': time.monotonic()
        })

        self.protocol.send_data(status_data)

    def log_data(self, sensor_data):
        """Log data locally"""
        self.data_history.append(sensor_data)

        # Keep only last 1000 readings to save memory
        if len(self.data_history) > 1000:
            self.data_history = self.data_history[-1000:]

    def run(self):
        """Main system loop"""
        print("Starting integrated monitoring system...")
        print("Communicating with Raspberry Pi 4 via UART")
        print("Press Ctrl+C to stop")

        try:
            while True:
                current_time = time.monotonic()

                # Read all sensors
                sensor_data = self.read_all_sensors()

                # Process commands from Pi 4
                self.process_commands()

                # Safety checks
                self.safety_checks(sensor_data)

                # Run control loop
                self.run_control_loop(sensor_data)

                # Send data to Pi 4 and log locally
                if (current_time - self.last_log_time) >= self.log_interval:
                    self.send_data_to_pi4(sensor_data)
                    self.log_data(sensor_data)

                    # Print status
                    liquid_temp = sensor_data.get('liquid_temperature', 0)
                    co2_ppm = sensor_data.get('co2_ppm', 0)
                    humidity = sensor_data.get('humidity', 0)
                    heater_power = sensor_data.get('heater_power', 0)

                    print(f"Liquid: {liquid_temp:.1f}°C | CO2: {co2_ppm:.0f}ppm | "
                          f"RH: {humidity:.1f}% | Heat: {heater_power:.0f}% | "
                          f"Target: {self.target_temperature:.1f}°C")

                    self.last_log_time = current_time

                time.sleep(0.5)  # 500ms loop

        except KeyboardInterrupt:
            print("\nShutdown requested...")
        finally:
            self.dimmer.set_power(0)
            print("System safely shut down")

# Main execution
def main():
    """Main function"""

    # Pin assignments
    TEMP_SENSOR_PIN = board.GP2     # DS18B20 liquid temperature
    DIMMER_OUTPUT_PIN = board.GP3   # RBDDimmer control
    DIMMER_ZC_PIN = None            # Zero-cross detection (optional)
    I2C_SDA = board.GP0             # SCD30 I2C data
    I2C_SCL = board.GP1             # SCD30 I2C clock
    UART_TX = board.GP4             # To Pi 4 RX
    UART_RX = board.GP5             # From Pi 4 TX

    try:
        # Initialize system
        system = IntegratedMonitoringSystem(
            temp_pin=TEMP_SENSOR_PIN,
            dimmer_pin=DIMMER_OUTPUT_PIN,
            dimmer_zc_pin=DIMMER_ZC_PIN,
            i2c_sda=I2C_SDA,
            i2c_scl=I2C_SCL,
            uart_tx=UART_TX,
            uart_rx=UART_RX
        )

        # Run main loop
        system.run()

    except Exception as e:
        print(f"System error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()