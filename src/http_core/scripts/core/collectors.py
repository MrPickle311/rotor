from dataclasses import dataclass
from typing import Dict


@dataclass
class DroneStateCollector:
    drone_longitude: float
    drone_lattitude: float
    drone_altitude: float
    drone_battery_voltage: float
    drone_battery_temperature: float


@dataclass
class RotorStateCollector:
    position: float
    state: int


class ComStateCollector:
    drone_state = DroneStateCollector(0.0, 0.0, 0.0, 0.0, 0.0)
    rotor_state = RotorStateCollector(0.0, -1)
