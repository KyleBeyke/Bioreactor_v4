import sys
import types
from pathlib import Path

# Stub hardware-specific modules before importing the module under test
for name in ["board", "busio", "digitalio", "adafruit_scd30"]:
    sys.modules[name] = types.ModuleType(name)

onewire = types.ModuleType("onewire")
onewire.OneWire = object
sys.modules["onewire"] = onewire

ds18x20 = types.ModuleType("ds18x20")
ds18x20.DS18X20 = object
sys.modules["ds18x20"] = ds18x20

# Ensure the project root is on the import path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from biodigester_control_loop import SimpleBiodigester


def make_controller():
    ctrl = SimpleBiodigester.__new__(SimpleBiodigester)
    ctrl.active = True
    ctrl.emergency_stop = False
    ctrl.current_profile = {"max_power": 40}
    return ctrl


def test_no_heating_when_above_target():
    ctrl = make_controller()
    assert ctrl.calculate_power(-0.1) == 0
    assert ctrl.calculate_power(0.0) == 0


def test_power_increases_with_positive_error():
    ctrl = make_controller()
    assert ctrl.calculate_power(0.2) == 5
    assert ctrl.calculate_power(1.0) == 15
    assert ctrl.calculate_power(3.0) == 25
    assert ctrl.calculate_power(10.0) == 35
