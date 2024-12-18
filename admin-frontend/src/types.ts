export type NodeId = number;
export type LatLon = [number, number];

export const BAD_POSITION: LatLon = [0.0, 0.0];

export type Drone = {
    drone_id: number,
    base_station_node: NodeInfo,
    battery_life_m: number
}

export type AppSettings = {
    map_lat_lon: LatLon,
    map_radius: number,
    edges_to_visit: EdgeInfo[],
    drones: Drone[]
}

export const DEFAULT_SETTINGS: AppSettings = {
    map_lat_lon: [0.0, 0.0],
    map_radius: 1500.0,
    edges_to_visit: [],
    drones: []
}

export type NodeInfo = {
  nodeId: number,
  position: LatLon,
}

export type EdgeInfo = {
  fromNodeId: number,
  toNodeId: number,
  positions: LatLon[],
}

export type AppSettingsFromApi = {
    map_lat_lon: LatLon,
    map_radius: number,
    edges_to_visit: [NodeId, NodeId, boolean][],
    drones: {
        drone_id: number,
        base_station_id: NodeId,
        battery_life_m: number
    }[]
}
export type PlanningOutput = {[droneId: number]: {positions: LatLon[]}};