from flask import Flask

app = Flask(__name__)

app.config['MAX_CONTENT_LENGTH'] = 2**26

@app.route('/hello/<name>')
def hello_name(name):
   return 'Hello %s!' % name

@app.route('/api/v1/floodzones', methods=['GET'])
def floodzones():
    pass

@app.route('/api/v1/upload', methods=['POST'])
def upload():
    img = Image.open(BytesIO(request.get_data()))
    evaluator = app_state.get_evaluator()
    res, conf = evaluator.evaluate_one(img)
    pass

if __name__ == '__main__':
   app.run()
