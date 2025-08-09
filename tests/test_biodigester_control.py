import sys
import types
from pathlib import Path

# Ensure project root is on the Python path
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

# Create minimal stub modules to satisfy imports in biodigester_control_loop
sys.modules.setdefault('board', types.ModuleType('board'))
sys.modules.setdefault('digitalio', types.SimpleNamespace(DigitalInOut=lambda pin: None,
                                                        Direction=types.SimpleNamespace(OUTPUT=None)))
sys.modules.setdefault('busio', types.SimpleNamespace(I2C=lambda scl, sda: None,
                                                     UART=lambda tx, rx, baudrate: None))
# Provide dummy classes for sensor libraries
sys.modules.setdefault('onewire', types.SimpleNamespace(OneWire=lambda pin: None))
sys.modules.setdefault('ds18x20', types.SimpleNamespace(DS18X20=lambda ow: None))
sys.modules.setdefault('adafruit_scd30', types.SimpleNamespace(SCD30=lambda i2c: None))

import biodigester_control_loop
from biodigester_control_loop import SimpleBiodigester

import pytest


@pytest.fixture
def biodigester():
    """Return a SimpleBiodigester instance with hardware init patched out."""
    # Patch hardware initialization methods to avoid hardware access
    SimpleBiodigester._init_temperature_sensor = lambda self, pin: None
    SimpleBiodigester._init_co2_sensor = lambda self, sda, scl: None
    SimpleBiodigester._init_dimmer = lambda self, pin: None
    SimpleBiodigester._init_uart = lambda self, tx, rx: None

    bd = SimpleBiodigester(1, 2, 3, 4, 5, 6)
    bd.active = True  # Enable power calculation
    return bd


def test_calculate_power_profiles(biodigester):
    """Power output should follow profile-based limits for temperature errors."""
    assert biodigester.calculate_power(0.3) == 5
    assert biodigester.calculate_power(1.0) <= 15
    assert biodigester.calculate_power(3.0) <= 25
    assert biodigester.calculate_power(6.0) <= 35
