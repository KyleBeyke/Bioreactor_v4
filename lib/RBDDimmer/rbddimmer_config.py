"""
RBDDimmer Configuration and Testing Module
Provides configuration tools, safety profiles, and testing functions
"""

import time
import board
import rbddimmer

class DimmerConfig:
    """Configuration profiles for different crucible heating applications"""

    # Predefined heating profiles
    PROFILES = {
        'safe_test': {
            'max_power': 25,
            'ramp_rate': 2.0,
            'description': 'Safe testing profile - limited power'
        },
        'aluminum_melting': {
            'max_power': 80,
            'ramp_rate': 5.0,
            'max_temp_rate': 8,
            'target_temp': 700,
            'description': 'Aluminum melting (660°C) with controlled ramp'
        },
        'silver_melting': {
            'max_power': 90,
            'ramp_rate': 3.0,
            'max_temp_rate': 6,
            'target_temp': 980,
            'description': 'Silver melting (962°C) - high temperature'
        },
        'annealing': {
            'max_power': 60,
            'ramp_rate': 1.0,
            'max_temp_rate': 3,
            'target_temp': 500,
            'description': 'Metal annealing - slow controlled heating'
        },
        'glass_working': {
            'max_power': 85,
            'ramp_rate': 4.0,
            'max_temp_rate': 7,
            'target_temp': 800,
            'description': 'Glass melting and working'
        }
    }

    @classmethod
    def get_profile(cls, profile_name):
        """Get configuration profile by name"""
        return cls.PROFILES.get(profile_name, cls.PROFILES['safe_test'])

    @classmethod
    def list_profiles(cls):
        """List all available profiles"""
        for name, config in cls.PROFILES.items():
            print(f"{name}: {config['description']}")

class DimmerTester:
    """Testing and calibration tools for RBDDimmer"""

    def __init__(self, dimmer):
        self.dimmer = dimmer
        self.test_results = []

    def basic_functionality_test(self):
        """Test basic dimmer functionality"""
        print("Running basic functionality test...")

        tests = [
            ("Power Off", 0),
            ("Low Power", 10),
            ("Medium Power", 50),
            ("High Power", 80),
            ("Power Off", 0)
        ]

        results = []

        for test_name, power in tests:
            print(f"  {test_name}: {power}%")

            success = self.dimmer.set_power(power)
            time.sleep(2)  # Hold for 2 seconds

            actual_power = self.dimmer.get_power()
            status = self.dimmer.get_status()

            result = {
                'test': test_name,
                'requested_power': power,
                'actual_power': actual_power,
                'success': success and abs(actual_power - power) < 0.1,
                'emergency_stop': status['emergency_stop']
            }

            results.append(result)
            print(f"    Result: {'PASS' if result['success'] else 'FAIL'}")

        self.test_results.extend(results)
        return all(r['success'] for r in results)

    def ramp_test(self, start_power=0, end_power=50, ramp_time=10):
        """Test power ramping functionality"""
        print(f"Testing power ramp from {start_power}% to {end_power}% over {ramp_time}s")

        # Set starting power
        self.dimmer.set_power(start_power)
        time.sleep(1)

        # Start ramp
        start_time = time.monotonic()
        self.dimmer.set_power_ramp(end_power, ramp_time)

        # Monitor ramping
        samples = []
        while self.dimmer.mode == rbddimmer.RAMP_MODE:
            current_time = time.monotonic() - start_time
            current_power = self.dimmer.get_power()

            samples.append({
                'time': current_time,
                'power': current_power
            })

            print(f"  Time: {current_time:.1f}s, Power: {current_power:.1f}%")
            time.sleep(0.5)
            self.dimmer.update()

            # Safety timeout
            if current_time > ramp_time + 5:
                break

        # Analyze results
        final_power = self.dimmer.get_power()
        actual_time = samples[-1]['time'] if samples else 0

        success = (abs(final_power - end_power) < 1.0 and
                  abs(actual_time - ramp_time) < 2.0)

        print(f"  Ramp test: {'PASS' if success else 'FAIL'}")
        print(f"  Final power: {final_power:.1f}% (target: {end_power}%)")
        print(f"  Actual time: {actual_time:.1f}s (target: {ramp_time}s)")

        return success, samples

    def safety_test(self):
        """Test safety features"""
        print("Testing safety features...")

        # Test emergency stop
        print("  Testing emergency stop...")
        self.dimmer.set_power(50)
        time.sleep(1)

        self.dimmer.emergency_stop()
        status = self.dimmer.get_status()

        emergency_test = (status['emergency_stop'] and
                         status['power_level'] == 0)

        print(f"    Emergency stop: {'PASS' if emergency_test else 'FAIL'}")

        # Test emergency reset
        self.dimmer.reset_emergency()
        status = self.dimmer.get_status()
        reset_test = not status['emergency_stop']

        print(f"    Emergency reset: {'PASS' if reset_test else 'FAIL'}")

        # Test max power limit
        print("  Testing max power limit...")
        original_max = self.dimmer.max_power
        self.dimmer.set_max_power(30)

        self.dimmer.set_power(50)  # Try to exceed limit
        actual_power = self.dimmer.get_power()
        limit_test = actual_power <= 30

        print(f"    Max power limit: {'PASS' if limit_test else 'FAIL'}")

        # Restore original max power
        self.dimmer.set_max_power(original_max)

        return emergency_test and reset_test and limit_test

    def zero_cross_test(self):
        """Test zero-crossing detection if available"""
        status = self.dimmer.get_status()

        if not status['zero_cross_available']:
            print("Zero-crossing detection not available - skipping test")
            return True

        print("Testing zero-crossing detection...")

        # Try calibration
        calibration_success = self.dimmer.calibrate_zero_cross()
        print(f"  Calibration: {'PASS' if calibration_success else 'FAIL'}")

        return calibration_success

    def generate_report(self):
        """Generate comprehensive test report"""
        print("\n" + "="*50)
        print("RBDDIMMER TEST REPORT")
        print("="*50)

        status = self.dimmer.get_status()

        print("Device Information:")
        print(f"  PWM Available: {status['pwm_available']}")
        print(f"  Zero-Cross Available: {status['zero_cross_available']}")
        print(f"  Max Power Setting: {status['max_power']}%")
        print(f"  Current Power: {status['power_level']:.1f}%")

        print("\nTest Results:")
        if self.test_results:
            passed = sum(1 for r in self.test_results if r['success'])
            total = len(self.test_results)
            print(f"  Tests Passed: {passed}/{total}")

            for result in self.test_results:
                status_str = "PASS" if result['success'] else "FAIL"
                print(f"    {result['test']}: {status_str}")
        else:
            print("  No tests completed")

        print("\nRecommendations:")
        if status['pwm_available']:
            print("  ✓ PWM dimming available - smooth power control")
        else:
            print("  ⚠ No PWM - using digital switching (may cause flicker)")

        if status['zero_cross_available']:
            print("  ✓ Zero-crossing detection available")
        else:
            print("  ⚠ No zero-crossing detection - basic timing only")

        print("="*50)

class CrucibleController:
    """High-level controller for crucible heating applications"""

    def __init__(self, dimmer_pin, zero_cross_pin=None, profile='safe_test'):
        """
        Initialize crucible controller

        Args:
            dimmer_pin: Pin connected to dimmer module
            zero_cross_pin: Zero-crossing detection pin (optional)
            profile: Heating profile name from DimmerConfig
        """
        # Load configuration profile
        self.config = DimmerConfig.get_profile(profile)

        # Initialize dimmer with profile settings
        self.dimmer = rbddimmer.RBDDimmer(
            dimmer_pin,
            zero_cross_pin,
            max_power=self.config['max_power']
        )

        self.dimmer.begin(safety_enabled=True)

        # Controller state
        self.target_temperature = 0
        self.current_temperature = 0
        self.heating_active = False

        print(f"Crucible controller initialized with '{profile}' profile")
        print(f"Description: {self.config['description']}")

    def start_heating(self, target_temp, current_temp=25):
        """
        Start controlled heating to target temperature

        Args:
            target_temp: Target temperature in °C
            current_temp: Current temperature in °C
        """
        self.target_temperature = target_temp
        self.current_temperature = current_temp

        # Calculate safe ramp time
        ramp_time = rbddimmer.calculate_crucible_ramp_time(
            current_temp, target_temp,
            self.config.get('max_temp_rate', 5)
        )

        # Calculate initial power based on temperature difference
        temp_diff = target_temp - current_temp
        initial_power = min(
            self.config['max_power'],
            max(10, temp_diff / 10)  # Rough estimate: 1% per 10°C
        )

        print(f"Starting heating: {current_temp}°C → {target_temp}°C")
        print(f"Estimated ramp time: {ramp_time/60:.1f} minutes")
        print(f"Initial power: {initial_power:.1f}%")

        # Start ramped heating
        self.dimmer.set_power_ramp(initial_power, ramp_time)
        self.heating_active = True

        return ramp_time

    def update_temperature(self, current_temp):
        """Update current temperature and adjust power if needed"""
        self.current_temperature = current_temp

        if not self.heating_active:
            return

        # Simple temperature-based power adjustment
        temp_error = self.target_temperature - current_temp

        if temp_error <= 0:
            # Target reached - reduce power
            self.dimmer.set_power_ramp(10, 30)  # Ramp down to 10% over 30s
        elif temp_error > 50:
            # Far from target - maintain heating
            pass  # Let ramp continue
        else:
            # Close to target - fine tune
            adjustment_power = min(
                self.config['max_power'],
                max(5, temp_error / 2)
            )
            self.dimmer.set_power(adjustment_power, immediate=False)

    def emergency_stop(self):
        """Emergency stop all heating"""
        self.dimmer.emergency_stop()
        self.heating_active = False
        print("EMERGENCY STOP - All heating stopped")

    def stop_heating(self):
        """Controlled stop of heating"""
        print("Stopping heating - ramping down...")
        self.dimmer.set_power_ramp(0, 60)  # Ramp down over 1 minute
        self.heating_active = False

    def get_status(self):
        """Get comprehensive controller status"""
        dimmer_status = self.dimmer.get_status()

        return {
            'target_temperature': self.target_temperature,
            'current_temperature': self.current_temperature,
            'heating_active': self.heating_active,
            'power_level': dimmer_status['power_level'],
            'target_power': dimmer_status['target_power'],
            'emergency_stop': dimmer_status['emergency_stop'],
            'profile': self.config['description'],
            'max_power': dimmer_status['max_power']
        }

    def update(self):
        """Update controller - call this frequently"""
        self.dimmer.update()

# Example usage and testing functions
def run_comprehensive_test(dimmer_pin=board.GP3, zero_cross_pin=board.GP4):
    """Run comprehensive test suite"""
    print("Starting comprehensive RBDDimmer test...")

    # Initialize dimmer
    dimmer = rbddimmer.RBDDimmer(dimmer_pin, zero_cross_pin, max_power=50)
    dimmer.begin(safety_enabled=True)

    # Create tester
    tester = DimmerTester(dimmer)

    try:
        # Run all tests
        print("\n1. Basic Functionality Test")
        basic_ok = tester.basic_functionality_test()

        print("\n2. Ramp Test")
        ramp_ok, samples = tester.ramp_test(0, 30, 15)

        print("\n3. Safety Test")
        safety_ok = tester.safety_test()

        print("\n4. Zero-Cross Test")
        zc_ok = tester.zero_cross_test()

        # Generate report
        tester.generate_report()

        overall_success = basic_ok and ramp_ok and safety_ok and zc_ok
        print(f"\nOverall Test Result: {'PASS' if overall_success else 'FAIL'}")

        return overall_success

    finally:
        # Cleanup
        dimmer.set_power(0)
        dimmer.deinit()

def crucible_heating_demo(dimmer_pin=board.GP3, zero_cross_pin=board.GP4):
    """Demonstrate crucible heating control"""
    print("Crucible Heating Control Demo")
    print("=" * 40)

    # Show available profiles
    print("Available heating profiles:")
    DimmerConfig.list_profiles()

    # Initialize with aluminum melting profile
    controller = CrucibleController(
        dimmer_pin, zero_cross_pin,
        profile='aluminum_melting'
    )

    try:
        # Simulate heating cycle
        print("\nStarting simulated aluminum melting cycle...")

        # Start heating
        ramp_time = controller.start_heating(target_temp=700, current_temp=25)

        # Simulate temperature readings over time
        simulation_time = 0
        time_step = 5  # 5 second steps

        while simulation_time < min(ramp_time, 300):  # Max 5 minutes for demo
            # Simulate temperature rise
            temp_rise_rate = 2.0  # °C per minute
            simulated_temp = 25 + (simulation_time / 60) * temp_rise_rate

            # Update controller
            controller.update_temperature(simulated_temp)
            controller.update()

            # Display status
            status = controller.get_status()
            print(f"Time: {simulation_time:3d}s | "
                  f"Temp: {simulated_temp:5.1f}°C | "
                  f"Power: {status['power_level']:5.1f}% | "
                  f"Target: {status['target_power']:5.1f}%")

            time.sleep(1)  # Actual wait (reduced for demo)
            simulation_time += time_step

        print("\nDemo complete - stopping heating")
        controller.stop_heating()

        # Wait for ramp down
        for i in range(10):
            controller.update()
            status = controller.get_status()
            print(f"Ramp down: {status['power_level']:.1f}%")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nDemo interrupted - emergency stop")
        controller.emergency_stop()

    finally:
        controller.dimmer.deinit()

def quick_power_test(dimmer_pin=board.GP3, power_level=25):
    """Quick test at specific power level"""
    print(f"Quick test at {power_level}% power")

    dimmer = rbddimmer.RBDDimmer(dimmer_pin, max_power=50)
    dimmer.begin()

    try:
        dimmer.set_power(power_level)

        for i in range(30):  # 30 seconds
            dimmer.update()
            status = dimmer.get_status()

            if i % 5 == 0:  # Print every 5 seconds
                print(f"Time: {i:2d}s | Power: {status['power_level']:5.1f}% | "
                      f"Mode: {status['mode']}")

            time.sleep(1)

    finally:
        dimmer.set_power(0)
        time.sleep(1)
        dimmer.deinit()

# Safety configuration functions
def create_custom_profile(name, max_power, ramp_rate, description, max_temp_rate=5):
    """Create custom heating profile"""
    profile = {
        'max_power': max_power,
        'ramp_rate': ramp_rate,
        'max_temp_rate': max_temp_rate,
        'description': description
    }

    DimmerConfig.PROFILES[name] = profile
    print(f"Created custom profile '{name}': {description}")
    return profile

def validate_setup(dimmer_pin=board.GP3, zero_cross_pin=board.GP4):
    """Validate hardware setup before use"""
    print("Validating RBDDimmer hardware setup...")

    try:
        # Test basic initialization
        dimmer = rbddimmer.RBDDimmer(dimmer_pin, zero_cross_pin, max_power=10)
        dimmer.begin()

        status = dimmer.get_status()
        print(f"✓ Dimmer initialized successfully")
        print(f"  PWM available: {status['pwm_available']}")
        print(f"  Zero-cross available: {status['zero_cross_available']}")

        # Test basic power control
        print("Testing basic power control...")
        dimmer.set_power(5)
        time.sleep(2)

        actual_power = dimmer.get_power()
        if abs(actual_power - 5) < 0.1:
            print("✓ Basic power control working")
        else:
            print("⚠ Power control issue detected")

        # Test safety features
        dimmer.emergency_stop()
        if dimmer.get_status()['emergency_stop']:
            print("✓ Emergency stop working")
        else:
            print("⚠ Emergency stop not working")

        dimmer.reset_emergency()
        dimmer.set_power(0)
        dimmer.deinit()

        print("✓ Hardware validation complete")
        return True

    except Exception as e:
        print(f"✗ Hardware validation failed: {e}")
        return False

if __name__ == "__main__":
    # Run validation by default
    validate_setup()