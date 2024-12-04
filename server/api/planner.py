from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import networkx
import logging
import osmnx
import numpy as np
import scipy
from dataclasses import dataclass

import utils
import drone

DEFAULT_LOGGER: logging.Logger = logging.getLogger()

def _make_path_map(
    G: networkx.MultiGraph,
    points: list[int]
) -> dict[tuple[int, int], tuple[list[int], float]]:
    n = len(points)
    return {
        (source, dest): (
            path,
            sum([next(iter(G.get_edge_data(sn, dn).values()))['length'] for sn, dn in zip(path[:-1], path[1:])])
        )
        for source, dest, path in zip(np.repeat(points, n), np.tile(points, n), osmnx.routing.shortest_path(G, np.repeat(points, n), np.tile(points, n), cpus=4))
    }

def _cull_and_symmetrize(G: networkx.MultiDiGraph, key_nodes: list[int], leeway: float = 0.025) -> networkx.MultiGraph:
    nodes_df, edges_df = osmnx.convert.graph_to_gdfs(G)
    key_points_xy = nodes_df.loc[key_nodes][['x', 'y']].to_numpy()
    ch = scipy.spatial.ConvexHull(key_points_xy)
    all_points_xy = nodes_df[['x', 'y']].to_numpy()
    is_inside = np.all(np.sum(ch.equations[np.newaxis, :, :2] * all_points_xy[:, np.newaxis, :], axis=2) < -ch.equations[np.newaxis, :, 2] + leeway, axis=1)
    selected_nodes_id = list(nodes_df[is_inside].index)
    sg = osmnx.convert.to_undirected(G)
    sg = networkx.induced_subgraph(sg, selected_nodes_id)
    return sg.copy()

@dataclass
class PlannedPath:
    path_length: float
    lats: list[float]
    lons: list[float]

def plan_drones_path(
    orig_G: networkx.MultiDiGraph,
    edges: list[tuple[int, int, bool]],
    drones: list[drone.Drone],
    scale_factor: float = 1000,
) -> dict[int, PlannedPath]:
    logger = logging.getLogger("drone-planner")

    G = _cull_and_symmetrize(orig_G, list(set([p for (u, v, bidi) in edges for p in (u, v)])))

    forward_edges = [(fro, to) for (fro, to, bidi) in edges]
    backward_edges = [(to, fro) for (fro, to, bidi) in edges if bidi]
    
    forward_backward_indices = [i for i,(fro, to, bidi) in enumerate(edges) if bidi]
    
    all_edges = forward_edges + backward_edges
    n_all_edges = len(all_edges)
    # depot_nodes_raw = osmnx.distance.nearest_nodes(G, X=[d.base_station_lat_lon[1] for d in drones], Y=[d.base_station_lat_lon[0] for d in drones])
    depot_nodes_raw = [d.base_station_id for d in drones]
    depot_nodes = sorted(list(set(depot_nodes_raw)))
    AVOID_DEPOT_NODES = 10
    points = depot_nodes + [p for (u, v, bidi) in edges for p in (u, v)]
    points = list(set(points))
    logging.debug("making distance map.")
    path_map = _make_path_map(G, points)
    logging.debug("made distance map.")

    def distance_between(idx1, idx2):
        if idx1 == idx2: return 0.0
        d = 0.0
        if (idx1 < n_all_edges):
            d += G.get_edge_data(*all_edges[idx1], key=0)["length"] / 2
            n1 = all_edges[idx1][1]
        else:
            n1 = depot_nodes[idx1 - n_all_edges]
            d += AVOID_DEPOT_NODES
        if (idx2 < n_all_edges):
            d += G.get_edge_data(*all_edges[idx2], key=0)["length"] / 2
            n2 = all_edges[idx2][0]
        else:
            n2 = depot_nodes[idx2 - n_all_edges]
            d += AVOID_DEPOT_NODES
        _, pl = path_map[(n1, n2)]
        return d + pl
    
    num_nodes = len(all_edges) + len(depot_nodes)
    num_drones = len(drones)
    depot_idxs = [n_all_edges + depot_nodes.index(node) for node in depot_nodes_raw]

    manager = pywrapcp.RoutingIndexManager(
        num_nodes, num_drones, depot_idxs, depot_idxs
    )
    
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        dist = int(distance_between(from_node, to_node) * scale_factor)
        return dist

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    routing.AddDimensionWithVehicleCapacity(
        transit_callback_index,
        0,  # null capacity slack
        [int(d.battery_life_m * scale_factor) for d in drones],
        True,  # start cumul to zero
        "Distance",
    )
    
    distance_dimension = routing.GetDimensionOrDie("Distance")
    distance_dimension.SetGlobalSpanCostCoefficient(100)
    
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    for b_index, f_index in enumerate(forward_backward_indices):
        routing.AddDisjunction([f_index, b_index + len(forward_edges)], -1, 1)
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        #routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
    )
    search_parameters.local_search_metaheuristic = (
        #routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC
    )
    search_parameters.time_limit.FromSeconds(10)
    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if not solution:
        logger.error(f"no solution found (status={routing.status()}).")
        raise ValueError("no solution found")

    # extract routes from solution
    unfilled_routes = []
    for drone_id in range(num_drones):
        route_distance = 0.0
        index = routing.Start(drone_id)
        start_ni = manager.IndexToNode(index)
        assert start_ni >= n_all_edges
        this_route = [(None, depot_nodes[start_ni - n_all_edges])]
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            ni = manager.IndexToNode(index)
            if ni >= n_all_edges:
                assert routing.IsEnd(index)
                assert ni == start_ni
                break
            # assert ni < n_all_edges
            this_route.append(all_edges[ni])
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, drone_id
            )
        this_route.append((depot_nodes[start_ni - n_all_edges], None))
        unfilled_routes.append(this_route)
    visited_edges = set([tuple(sorted(e)) for u in unfilled_routes for e in u[1:-1]])
    wanted_edges = set([tuple(sorted([u, v])) for (u, v, bidi) in edges])

    if wanted_edges != visited_edges:
        logger.error(f"solution incomplete (only got {visited_edges} out of {wanted_edges} edges).")
        raise ValueError("incomplete solution")

    paths = []
    path_lengths = []
    output = dict()
    nodes_df = osmnx.graph_to_gdfs(G, nodes=True, edges=False)
    for drone, route in zip(drones, unfilled_routes):
        this_route = []
        drone_length = 0.0
        for (u, v) in route[1:-1]:
            segment, l = path_map[(u, v)]
            drone_length += l
        for (_, u), (v, _) in zip(route[:-1], route[1:]):
            segment, l = path_map[(u, v)]
            drone_length += l
            this_route += segment
        paths.append(utils.remove_consecutive_dup(this_route))
        path_lengths.append(drone_length)

        output[drone.drone_id] = dict(
            path_length = drone_length,
            lats = nodes_df.loc[this_route]['y'].to_numpy().tolist(),
            lons = nodes_df.loc[this_route]['x'].to_numpy().tolist(),
        )
    # return output
    return {
        drone.drone_id: PlannedPath(path_length=length, lats=[y for (y, x) in lls], lons=[x for (y, x) in lls])
        for drone, lls, length in zip(drones, utils.fill_route(None, orig_G, paths), path_lengths)
    }
