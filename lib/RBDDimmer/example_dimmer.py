# example_dimmer.py
"""
Example usage of the RBDDimmer CircuitPython library

This example demonstrates:
1. Basic dimming control
2. Ramping (fade) effects
3. Reading potentiometer for manual control
4. Multiple dimmer control
"""

import board
import time
import analogio
import rbddimmer

# Example 1: Basic Dimming
def example_basic():
    """Basic dimmer control example"""
    print("Basic Dimmer Example")

    # Initialize dimmer
    # Connect: PWM/Output to D5, Zero-Cross to D2
    dimmer = rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60)
    dimmer.begin()

    try:
        while True:
            # Cycle through different power levels
            for power in [0, 25, 50, 75, 100, 75, 50, 25, 0]:
                print(f"Setting power to {power}%")
                dimmer.set_power(power)

                # Update dimmer for 2 seconds
                end_time = time.monotonic() + 2.0
                while time.monotonic() < end_time:
                    dimmer.update()
                    time.sleep(0.001)  # 1ms update rate

    except KeyboardInterrupt:
        print("Shutting down...")
        dimmer.off()
        dimmer.deinit()

# Example 2: Smooth Ramping
def example_ramping():
    """Smooth fade/ramping example"""
    print("Ramping Example")

    dimmer = rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60)
    dimmer.begin()

    try:
        while True:
            # Fade up to 100% over 3 seconds
            print("Fading up...")
            dimmer.set_power_ramp(100, 3000)

            # Wait for ramp to complete
            while dimmer.get_power() != 100:
                dimmer.update()
                time.sleep(0.001)

            time.sleep(1)  # Hold at 100%

            # Fade down to 0% over 3 seconds
            print("Fading down...")
            dimmer.set_power_ramp(0, 3000)

            # Wait for ramp to complete
            while dimmer.get_power() != 0:
                dimmer.update()
                time.sleep(0.001)

            time.sleep(1)  # Hold at 0%

    except KeyboardInterrupt:
        print("Shutting down...")
        dimmer.off()
        dimmer.deinit()

# Example 3: Potentiometer Control
def example_pot_control():
    """Control dimmer with potentiometer"""
    print("Potentiometer Control Example")

    # Initialize dimmer
    dimmer = rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60)
    dimmer.begin()

    # Initialize potentiometer on A0
    pot = analogio.AnalogIn(board.A0)

    def get_pot_percentage():
        """Convert pot reading to percentage"""
        return int((pot.value / 65535) * 100)

    try:
        last_power = -1

        while True:
            # Read potentiometer
            pot_power = get_pot_percentage()

            # Only update if changed significantly (reduce noise)
            if abs(pot_power - last_power) > 2:
                print(f"Power: {pot_power}%")
                dimmer.set_power(pot_power)
                last_power = pot_power

            # Always update dimmer
            dimmer.update()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Shutting down...")
        dimmer.off()
        dimmer.deinit()
        pot.deinit()

# Example 4: Multiple Dimmers
def example_multiple_dimmers():
    """Control multiple dimmers independently"""
    print("Multiple Dimmers Example")

    # Initialize multiple dimmers
    dimmer1 = rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60)
    dimmer2 = rbddimmer.RBDDimmer(board.D6, board.D3, frequency=60)

    dimmer1.begin()
    dimmer2.begin()

    try:
        while True:
            # Dimmer 1: Sine wave pattern
            # Dimmer 2: Inverse sine wave pattern
            for i in range(360):
                import math

                # Calculate power levels
                power1 = int(50 + 50 * math.sin(math.radians(i)))
                power2 = int(50 + 50 * math.sin(math.radians(i + 180)))

                dimmer1.set_power(power1)
                dimmer2.set_power(power2)

                # Update both dimmers
                for _ in range(10):  # Update for ~10ms
                    dimmer1.update()
                    dimmer2.update()
                    time.sleep(0.001)

    except KeyboardInterrupt:
        print("Shutting down...")
        dimmer1.off()
        dimmer2.off()
        dimmer1.deinit()
        dimmer2.deinit()

# Example 5: Light Sensor Auto-Dimming
def example_auto_dimming():
    """Automatic dimming based on ambient light"""
    print("Auto-Dimming Example")

    dimmer = rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60)
    dimmer.begin()

    # Light sensor on A1
    light_sensor = analogio.AnalogIn(board.A1)

    # Calibration values (adjust for your sensor)
    DARK_READING = 1000    # ADC reading in dark
    BRIGHT_READING = 50000 # ADC reading in bright light

    def map_range(x, in_min, in_max, out_min, out_max):
        """Map value from one range to another"""
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    try:
        while True:
            # Read light level
            light_level = light_sensor.value

            # Map to inverse power (darker = more light)
            target_power = int(map_range(
                light_level,
                DARK_READING,
                BRIGHT_READING,
                100,  # Full power when dark
                10    # Low power when bright
            ))

            # Clamp to valid range
            target_power = max(0, min(100, target_power))

            # Use ramping for smooth transitions
            if abs(dimmer.get_power() - target_power) > 5:
                print(f"Light: {light_level}, Power: {target_power}%")
                dimmer.set_power_ramp(target_power, 500)  # 0.5 second ramp

            dimmer.update()
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Shutting down...")
        dimmer.off()
        dimmer.deinit()
        light_sensor.deinit()

# Example 6: Theater/Disco Effect
def example_theater_effect():
    """Theater chase and strobe effects"""
    print("Theater Effect Example")

    # Multiple dimmers for different lights
    dimmers = [
        rbddimmer.RBDDimmer(board.D5, board.D2, frequency=60),
        rbddimmer.RBDDimmer(board.D6, board.D2, frequency=60),
        rbddimmer.RBDDimmer(board.D9, board.D2, frequency=60),
    ]

    for dimmer in dimmers:
        dimmer.begin()

    try:
        # Chase effect
        print("Chase effect...")
        for _ in range(10):  # 10 cycles
            for i, dimmer in enumerate(dimmers):
                # Turn on one at a time
                for j, d in enumerate(dimmers):
                    d.set_power(100 if i == j else 0)

                # Update all dimmers
                end_time = time.monotonic() + 0.2
                while time.monotonic() < end_time:
                    for d in dimmers:
                        d.update()
                    time.sleep(0.001)

        # Strobe effect
        print("Strobe effect...")
        for _ in range(20):  # 20 flashes
            # All on
            for dimmer in dimmers:
                dimmer.set_power(100)

            end_time = time.monotonic() + 0.05
            while time.monotonic() < end_time:
                for d in dimmers:
                    d.update()
                time.sleep(0.001)

            # All off
            for dimmer in dimmers:
                dimmer.set_power(0)

            end_time = time.monotonic() + 0.05
            while time.monotonic() < end_time:
                for d in dimmers:
                    d.update()
                time.sleep(0.001)

        # Fade all off
        for dimmer in dimmers:
            dimmer.set_power_ramp(0, 1000)

        # Wait for fade
        while any(d.get_power() > 0 for d in dimmers):
            for d in dimmers:
                d.update()
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Shutting down...")
        for dimmer in dimmers:
            dimmer.off()
            dimmer.deinit()

# Main menu
def main():
    """Run examples based on user selection"""
    examples = {
        '1': ('Basic Dimming', example_basic),
        '2': ('Smooth Ramping', example_ramping),
        '3': ('Potentiometer Control', example_pot_control),
        '4': ('Multiple Dimmers', example_multiple_dimmers),
        '5': ('Auto-Dimming', example_auto_dimming),
        '6': ('Theater Effects', example_theater_effect),
    }

    while True:
        print("\nRBDDimmer Examples:")
        for key, (name, _) in examples.items():
            print(f"{key}. {name}")
        print("Q. Quit")

        choice = input("\nSelect example: ").strip().upper()

        if choice == 'Q':
            break
        elif choice in examples:
            examples[choice][1]()
        else:
            print("Invalid choice")

if __name__ == "__main__":
    main()