from flask import current_app as app, g, request

from typing import Optional
from dataclasses import dataclass

import networkx
import osmnx

import floodzones
# import evaluate
import planner
import drone

@dataclass
class AppSettings:
    map_lat_lon: tuple[float, float]
    map_radius: float
    edges_to_visit: list[tuple[int, int]]
    drones: list[drone.Drone]

def default_app_settings() -> AppSettings:
    return AppSettings(map_lat_lon=(29.717997,-95.400547), map_radius=1500, edges_to_visit=[], drones=[])

def _create_map_graph(lat_lon: tuple[float, float], dist_m: float, **osmnx_kwargs) -> networkx.MultiDiGraph:
    return osmnx.graph_from_point(lat_lon, dist=dist_m)

class AppState:
    def __init__(self):
        self.settings: app_settings.AppSettings = default_app_settings()
        self.flood_db: floodzones.FloodDatabase = floodzones.FloodDatabase()
        # self.evaluator: evaluate.FloodEvaluator = evaluate.FloodEvaluator()
        self.map_graph: networkx.MultiDiGraph = _create_map_graph(self.settings.map_lat_lon, self.settings.map_radius)
        self.planned_path: Optional[dict[int, list[int]]] = None

        # HARDCODE SOME PARAMETERS BECAUSE WE DON'T HAVE FRONTEND ADMIN PANEL YET
        # TODO: remove this hardcoding
        INNER_LOOP_NODES = [7459593424, 7459595014, 2611472516, 151674436, 11529033887, 151764718, 2584454341, 151786655, 1019275181, 8406688429, 2611731387, 583250666, 5265110642, 569257577, 151421265, 11576321623, 151786632, 151378395, 2611731328, 1016949780, 2611731326, 151378198, 151786621, 569254436, 5255143591, 2584454318, 5255143589, 1018674570, 5255134086, 1018674551, 5255134079, 5255134076, 5572595732, 11529033886, 5572595105, 5255134077, 8627037753, 10772010783, 4706040461, 6906798197, 6906798195, 7459593472, 10772010792, 7459593473, 7459593483, 7459593424]
        MAIN_STREET_NODES = [1229920388, 151378245, 2161483950, 10772010844, 2161483947, 2611731375, 2161483953]
        from random import shuffle
        self.settings.edges_to_visit = (
            [(fr, to, True) for (fr, to) in zip(INNER_LOOP_NODES[:-1], INNER_LOOP_NODES[1:])] +
            [(fr, to, True) for (fr, to) in zip(MAIN_STREET_NODES[:-1], MAIN_STREET_NODES[1:])]
        )
        shuffle(self.settings.edges_to_visit)
        self.settings.drones = [drone.Drone(drone_id=drone_id, base_station_lat_lon=(29.717070,-95.400457), battery_life_m=2500) for drone_id in [123, 456, 789]]

    def plan_round(self):
        self.map_graph = _create_map_graph(self.settings.map_lat_lon, self.settings.map_radius)
        self.planned_path = planner.plan_drones_path(
            G=self.map_graph,
            edges=self.settings.edges_to_visit,
            drones=self.settings.drones,
            scale_factor=1000,
        )

def get_state() -> AppState:
    if 'app_state' not in g:
        g.app_state = AppState()
    return g.app_state
