from flask import current_app as app, g, request

import floodzones
import evaluate

def get_flood_db() -> floodzones.FloodDatabase:
    if 'flood_db' not in g:
        g.flood_db = floodzones.FloodDatabase()
    return g.flood_db

def get_evaluator() -> evaluate.FloodEvaluator:
   if 'flood_evaluator' not in g:
      g.flood_evaluator = evaluate.FloodEvaluator(model_path = app.config['EVAL_MODEL_PATH'])
   return g.flood_evaluator