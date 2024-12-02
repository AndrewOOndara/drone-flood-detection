from dataclasses import dataclass

@dataclass
class Drone:
    drone_id: int
    base_station_lat_lon: tuple[float, float]
    battery_life_m: float