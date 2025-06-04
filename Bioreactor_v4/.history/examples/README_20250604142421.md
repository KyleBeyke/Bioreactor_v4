# Integrated CO2 and Temperature Monitoring System Setup Guide

## System Overview

This system combines:
- **Raspberry Pi Pico** (CircuitPython) - Real-time sensor reading and control
- **Raspberry Pi 4** (Python) - Command interface, data logging, and visualization
- **DS18B20** - Liquid temperature measurement
- **SCD30** - CO2, ambient temperature, and humidity
- **RBDDimmer** - Crucible heating element control

## Hardware Connections

### Raspberry Pi Pico Wiring

```
DS18B20 Temperature Sensor:
- Yellow (Data) → GP2
- Red (VCC)     → 3.3V
- Black (GND)   → GND

SCD30 CO2 Sensor (I2C):
- VCC → 3.3V
- GND → GND
- SDA → GP0
- SCL → GP1

RBDDimmer:
- Control Signal → GP3
- Zero-Cross (optional) → GP4

UART to Raspberry Pi 4:
- TX (Pico GP4) → RX (Pi4 GPIO15/Pin 10)
- RX (Pico GP5) → TX (Pi4 GPIO14/Pin 8)
- GND → GND (shared ground)
```

### Raspberry Pi 4 Setup

Enable UART communication:
```bash
# Edit /boot/config.txt
sudo nano /boot/config.txt

# Add these lines:
enable_uart=1
dtoverlay=disable-bt

# Reboot
sudo reboot
```

## Software Installation

### Raspberry Pi Pico (CircuitPython)

1. **Install CircuitPython** on your Pico
2. **Install required libraries** in the `lib` folder:
   ```
   adafruit_scd30.mpy
   adafruit_onewire.mpy
   adafruit_ds18x20.mpy
   ```
3. **Copy the integrated monitoring code** to `code.py`

### Raspberry Pi 4 (Python)

1. **Install required packages**:
   ```bash
   pip install pyserial matplotlib pandas numpy
   ```

2. **Copy the control interface code** to your Pi 4

3. **Test serial connection**:
   ```bash
   # Check if UART is available
   ls /dev/ttyAMA0

   # Test basic communication
   python3 pi4_control_interface.py --mode interactive
   ```

## Getting Started

### Phase 1: Basic Setup and Testing

1. **Wire all components** according to the connection diagram
2. **Upload code to Pico** and verify it starts without errors
3. **Run the Pi 4 interface** in interactive mode:
   ```bash
   python3 pi4_control_interface.py --mode interactive
   ```
4. **Test communication** by checking status:
   ```
   crucible> status
   ```

### Phase 2: Sensor Verification

1. **Check temperature readings**:
   - DS18B20 should show liquid temperature
   - SCD30 should show ambient temperature, CO2, and humidity

2. **Calibrate CO2 sensor** (in fresh outdoor air):
   ```
   crucible> cal 400
   ```

3. **Verify heating control**:
   ```
   crucible> temp 30
   crucible> control on
   ```

### Phase 3: System Operation

1. **Set target temperature**:
   ```
   crucible> temp 100
   crucible> control on
   ```

2. **Monitor real-time data**:
   ```
   crucible> plot
   ```

3. **Adjust PID if needed**:
   ```
   crucible> pid 1.5 0.05 2.0
   ```

## Command Reference

### Interactive Commands

| Command | Description | Example |
|---------|-------------|---------|
| `temp <value>` | Set target temperature (°C) | `temp 80` |
| `control on/off` | Enable/disable temperature control | `control on` |
| `pid <kp> <ki> <kd>` | Set PID parameters | `pid 1.5 0.05 2.0` |
| `emergency` | Emergency stop | `emergency` |
| `reset` | Reset emergency stop | `reset` |
| `cal <ppm>` | Calibrate CO2 sensor | `cal 400` |
| `status` | Show current status | `status` |
| `plot` | Start real-time plotting | `plot` |
| `quit` | Exit interface | `quit` |

### Command Line Options

```bash
# Interactive mode (default)
python3 pi4_control_interface.py --mode interactive

# Real-time plotting
python3 pi4_control_interface.py --mode plot

# Automated logging
python3 pi4_control_interface.py --mode log --duration 24

# Set initial temperature and enable control
python3 pi4_control_interface.py --temp 100 --enable-control
```

## Safety Features

### Temperature Safety
- Maximum temperature limit (1200°C by default)
- Heating rate monitoring to prevent thermal shock
- Emergency shutdown capability
- Automatic heater shutdown on sensor failure

### CO2 Safety
- High CO2 concentration warnings (>10,000 ppm)
- Continuous monitoring and logging
- Real-time alerts

### System Safety
- Watchdog communication between Pico and Pi 4
- Automatic data logging and backup
- Emergency stop from either system

## Data Logging

### Automatic Logging
- Data logged every 5 seconds to CSV files
- Includes timestamps, all sensor readings, and control parameters
- Files automatically timestamped: `crucible_data_YYYYMMDD_HHMMSS.csv`

### Data Export
```bash
# Export last 24 hours of data
python3 -c "
from pi4_control_interface import ControlInterface
interface = ControlInterface()
interface.export_data(hours_back=24)
"
```

## Troubleshooting

### Common Issues

**No sensor readings:**
- Check wiring connections
- Verify power supply (3.3V)
- Check I2C connections for SCD30

**Communication errors:**
- Verify UART wiring between Pico and Pi 4
- Check that UART is enabled on Pi 4
- Ensure shared ground connection

**Temperature control not working:**
- Verify RBDDimmer connections
- Check that control is enabled: `control on`
- Ensure valid temperature setpoint

**CO2 readings seem incorrect:**
- Allow 10+ minutes for sensor warm-up
- Calibrate in fresh outdoor air (~400 ppm)
- Check sensor placement (avoid direct airflow)

### Debug Mode

Enable verbose output on Pico by modifying the main loop to print more diagnostic information.

## Advanced Features

### Custom PID Tuning
1. Start with conservative values: `pid 1.0 0.02 1.0`
2. Increase Kp for faster response
3. Add Ki for steady-state accuracy
4. Add Kd for stability

### Automated Experiments
Create scripts to run automated temperature profiles:

```python
from pi4_control_interface import ControlInterface
import time

interface = ControlInterface()

# Temperature ramp experiment
temps = [50, 75, 100, 125, 150]
for temp in temps:
    interface.set_temperature(temp)
    time.sleep(1800)  # 30 minutes at each temperature
```

### Data Analysis
Use pandas and matplotlib for advanced data analysis:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load logged data
df = pd.read_csv('crucible_data_20241201_120000.csv')
df['datetime'] = pd.to_datetime(df['datetime'])

# Plot temperature vs CO2
plt.figure(figsize=(12, 6))
plt.subplot(211)
plt.plot(df['datetime'], df['liquid_temperature'], label='Liquid Temp')
plt.plot(df['datetime'], df['target_temperature'], '--', label='Target')
plt.ylabel('Temperature (°C)')
plt.legend()

plt.subplot(212)
plt.plot(df['datetime'], df['co2_ppm'], color='purple')
plt.ylabel('CO2 (ppm)')
plt.xlabel('Time')
plt.show()
```

## Next Steps

1. **Test basic functionality** with all sensors connected
2. **Calibrate CO2 sensor** in known environment
3. **Run initial temperature control test** at moderate temperature
4. **Set up automated data logging** for your experiments
5. **Develop custom control scripts** for your specific applications

This system provides a solid foundation for monitoring CO2 emissions and controlling temperature in your crucible setup. The modular design allows easy expansion for additional sensors or control parameters.