export type NodeId = number;
export type LatLon = [number, number];

export type Drone = {
    drone_id: number,
    base_station_lat_lon: LatLon,
    battery_life_m: number
}

export type AppSettings = {
    map_lat_lon: LatLon,
    map_radius: number,
    edges_to_visit: [NodeId, NodeId, boolean][],
    drones: Drone[]
}

export const DEFAULT_SETTINGS: AppSettings = {
    map_lat_lon: [0.0, 0.0],
    map_radius: 1500.0,
    edges_to_visit: [],
    drones: []
}