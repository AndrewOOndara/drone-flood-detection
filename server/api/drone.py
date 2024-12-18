from dataclasses import dataclass

@dataclass
class Drone:
    drone_id: int
    base_station_id: int
    battery_life_m: float