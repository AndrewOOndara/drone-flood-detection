from flask import Flask, request, Response, jsonify
from flask_cors import CORS

import app_state
import floodzones
import utils
import time
from encoder import EnhancedJSONEncoder
import osmnx
import networkx

app = Flask(__name__)
CORS(app)

app.config['MAX_CONTENT_LENGTH'] = 2**26
app.json_encoder = EnhancedJSONEncoder

@app.route('/api/v1/zones', methods=['GET'])
def zones_endpoint():
    state = app_state.get_state()
    argslist = [request.args.get(k, type=float) for k in ('x0', 'y0', 'x1', 'y1')]
    if None in argslist: bounds = state.db.get_bounds()
    else: bounds = tuple(argslist)
    zones = state.flood_db.get_zones(bounds)
    return jsonify([
        vars(info) | dict(bbox=bbox)
        for (bbox, info) in zones
    ])

@app.route('/api/v1/upload', methods=['POST'])
def upload_endpoint():
    state = app_state.get_state()

    bounds = tuple([request.args.get(k, type=float) for k in ('x0', 'y0', 'x1', 'y1')])
    drone_id = request.args.get('drone_id', default=-1, type=int)
    now = time.time()
    img = Image.open(BytesIO(request.get_data()))
    res, conf = state.evaluator.evaluate_one(img)
    state.flood_db.update_zone(bounds, floodzones.ZoneDbEntry(time=now, drone_id=drone_id, flooded=bool(res), confidence=conf))
    return "OK"

@app.route('/api/v1/waypoints', methods=['GET'])
def waypoints_endpoint():
    state = app_state.get_state()
    state.plan_round()
    return jsonify(state.planned_path)

@app.route('/api/v1/settings', methods=['GET'])
def settings_get_endpoint():
    state = app_state.get_state()
    return jsonify(state.settings)

@app.route('/api/v1/settings', methods=['PUT'])
def settings_put_endpoint():
    state = app_state.get_state()
    content = request.get_json()
    state.settings = app_state.dict_to_app_settings(content)
    state.load_map()
    state.planned_path = None
    return "OK"

@app.route('/api/v1/edge_from_coords', methods=['POST'])
def get_edge_from_coords():
    content = request.get_json()
    [[from_lat, from_lon], [to_lat, to_lon]] = [content[k] for k in ['from', 'to']]
    state = app_state.get_state()
    G = state.map_graph
    [n1, n2] = osmnx.distance.nearest_nodes(G, [from_lon, to_lon], [from_lat, to_lat])
    path = osmnx.routing.shortest_path(G, n1, n2)

    [linecoords] = utils.fill_route(state.map_edges_gdf, state.map_graph, [path])

    return jsonify(dict(
        from_node = n1,
        to_node = n2,
        coords = linecoords
    ))

@app.route('/api/v1/edges_geometry', methods=['POST'])
def get_edges_geometry():
    content = request.get_json()
    n1s = content["n1s"]
    n2s = content["n2s"]
    state = app_state.get_state()
    G = state.map_graph
    paths = osmnx.routing.shortest_path(G, n1s, n2s)
    lines = utils.fill_route(state.map_edges_gdf, state.map_graph, paths)
    return jsonify(lines)

@app.route('/api/v1/nodes_pos', methods=['POST'])
def get_nodes_pos():
    ns = request.get_json()
    state = app_state.get_state()
    nodes = state.map_graph.nodes
    return jsonify([(v['y'], v['x']) for nid in ns if (v:=nodes[nid]) is not None])

@app.route('/api/v1/node_from_coords', methods=['POST'])
def get_node_from_coords():
    content = request.get_json()
    [lat, lon] = content
    state = app_state.get_state()
    G = state.map_graph
    nid = osmnx.distance.nearest_nodes(G, lon, lat)
    node = G.nodes[nid]
    return jsonify(dict(
        node_id=nid,
        position=[node['y'], node['x']]
    ))

@app.route('/api/v1/find_path', methods=['POST'])
def find_path():
    points_to_visit = [tuple(a) for a in request.get_json()]
    state = app_state.get_state()
    G = state.map_graph
    ids_to_visit = osmnx.distance.nearest_nodes(G, [lon for (lat, lon) in points_to_visit], [lat for (lat, lon) in points_to_visit])
    nodes_df, edges_df = osmnx.convert.graph_to_gdfs(G, nodes=True, edges=True)
    flood_db = state.flood_db
    def is_wet_func(geometry):
        return flood_db.is_path_wet(list(geometry.coords))

    wet_mask = edges_df["geometry"].apply(is_wet_func)
    dry_edges_df = edges_df[~wet_mask]
    new_G = osmnx.convert.graph_from_gdfs(nodes_df, dry_edges_df)
    paths = osmnx.routing.shortest_path(new_G, ids_to_visit[:-1], ids_to_visit[1:])
    lines = utils.fill_route(state.map_edges_gdf, state.map_graph, paths)
    merged_lines = [p for l in lines for p in l]
    return jsonify(merged_lines)

@app.route('/api/v1/converted_flood_points', methods=['POST'])
def set_flood():
    content = request.get_json()
    c = content['flooded_coordinates']
    radius = 3e-4
    state = app_state.get_state()
    for (lat, lon) in c:
        state.flood_db.update_zone([lon - radius, lat - radius, lon + radius, lat + radius], floodzones.ZoneDbEntry(time=time.time(), drone_id=1, flooded=True, confidence=0.5))

    return "OK"

if __name__ == '__main__':
   app.run()
