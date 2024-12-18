// import { queryForPositions } from "./overpass";
import { AppSettings, AppSettingsFromApi, BAD_POSITION, EdgeInfo, LatLon, NodeId, NodeInfo, PlanningOutput } from "./types";

const API_BASE = 'http://localhost:5000/api/v1';

export async function apiGetSettings(): Promise<AppSettings> {
    const req = fetch(
        API_BASE + '/settings',
        {
            method: 'GET'
        }
    );
    const res = await req;
    const out: AppSettingsFromApi = await res.json();
    const dronePos = await getNodesPos(out.drones.map(d => d.base_station_id));
    const edgePos = await getEdgesGeometry(out.edges_to_visit.map(e => e[0]), out.edges_to_visit.map(e => e[1]));
    return {
        map_lat_lon: out.map_lat_lon,
        map_radius: out.map_radius,
        drones: out.drones.map((d, i: number) => ({
            drone_id: d.drone_id,
            battery_life_m: d.battery_life_m,
            base_station_node: {
                nodeId: d.base_station_id,
                position: dronePos[i]
            }
        })),
        edges_to_visit: out.edges_to_visit.map(([u1, u2, _bidi], i) => ({
            fromNodeId: u1,
            toNodeId: u2,
            positions: edgePos[i]
        }))
    }
}
export async function saveSettings(settings: AppSettings): Promise<void> {
    const toSend: AppSettingsFromApi = {
        drones: settings.drones.map(d => ({
            base_station_id: d.base_station_node.nodeId,  
            battery_life_m: d.battery_life_m,
            drone_id: d.drone_id
        })),
        edges_to_visit: settings.edges_to_visit.map(e => ([e.fromNodeId, e.toNodeId, true])),
        map_lat_lon: settings.map_lat_lon,
        map_radius: settings.map_radius
    };
    const req = fetch(
        API_BASE + '/settings',
        {
            method: 'PUT',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(toSend)
        }
    );
    await req;
}

export async function getNodesPos(nodeIds: NodeId[]): Promise<LatLon[]> {
    const req = fetch(
        API_BASE + '/nodes_pos',
        {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(nodeIds)
        }
    );
    const res = await req;
    const out = await res.json();
    return out;
}
export async function getEdgesGeometry(froms: NodeId[], tos: NodeId[]): Promise<LatLon[][]> {
    const req = fetch(
        API_BASE + '/edges_geometry',
        {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ n1s: froms, n2s: tos })
        }
    );
    const res = await req;
    const out = await res.json();
    return out;
}
export async function getEdgeFromCoords(from: LatLon, to: LatLon): Promise<EdgeInfo> {
    const req = fetch(
        API_BASE + '/edge_from_coords',
        {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ from, to })
        }
    );
    const res = await req;
    const out = await res.json();
    return {
        fromNodeId: out['from_node'],
        toNodeId: out['to_node'],
        positions: out['coords']
    };
}
export async function getNodeFromCoords(ll: LatLon): Promise<NodeInfo> {
    const req = fetch(
        API_BASE + '/node_from_coords',
        {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(ll)
        }
    );
    const res = await req;
    const out = await res.json();
    return {
        nodeId: out['node_id'],
        position: out['position']
    };
}
export async function getPlan(): Promise<PlanningOutput> {
    const req = fetch(
        API_BASE + '/waypoints'
    );
    const res = await req;
    const resp: {[droneId: string]: {lats: number[], lons: number[]}} = await res.json();
    const out: PlanningOutput = {};
    for (const e of Object.entries(resp)) {
        const [droneId, p] = e;
        const numPoints = p.lats.length;
        const positions: LatLon[] = [];
        for (let i = 0; i < numPoints; i++) positions.push([p.lats[i], p.lons[i]])
        out[parseInt(droneId)] = {positions}
    }
    return out;

    // return Object.fromEntries(Object.entries(out).map(([droneId, p]) => {
    //     const numPoints = p.lats.length;
    //     const positions = [];
    //     for (let i = 0; i < numPoints; i++) positions.push([p.lats[i], p.lons[i]])
    //     return [parseInt(droneId), { positions }];
    // }));
}