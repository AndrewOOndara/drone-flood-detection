from flask import Flask, request, Response, jsonify
import app_state
import floodzones
import time

app = Flask(__name__)

app.config['MAX_CONTENT_LENGTH'] = 2**26

@app.route('/hello/<name>')
def hello_name(name):
    return 'Hello %s!' % name

@app.route('/api/v1/zones', methods=['GET'])
def zones_endpoint():
    state = app_state.get_state()

    argslist = [request.args.get(k, type=float) for k in ('x0', 'y0', 'x1', 'y1')]
    if None in argslist: bounds = state.db.get_bounds()
    else: bounds = tuple(argslist)
    zones = state.db.get_zones(bounds)
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

if __name__ == '__main__':
   app.run()
