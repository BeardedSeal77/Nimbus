# nimbus/main.py
import time
from flask import Flask, jsonify
import random

app = Flask(__name__)

@app.route('/calculate', methods=['GET'])
def calculate():
    num1 = random.randint(1, 100)
    num2 = random.randint(1, 100)
    result = num1 + num2
    return jsonify({'num1': num1, 'num2': num2, 'result': result})

@app.route('/health', methods=['GET'])
def health_check():
    """
    Simple health check endpoint
    """
    return jsonify({"status": "healthy"})

if __name__ == '__main__':
    # Add a small delay to ensure the container is fully started
    time.sleep(2)
    print("Starting Nimbus random number generator service...")
    app.run(host='0.0.0.0', port=5002, debug=True)