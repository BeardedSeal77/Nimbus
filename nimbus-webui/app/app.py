from flask import Flask, render_template, jsonify
import requests
import os

app = Flask(__name__)

# Get the nimbus service hostname from environment or default to the service name in docker-compose
NIMBUS_HOST = os.environ.get('NIMBUS_HOST', 'nimbus')
NIMBUS_PORT = os.environ.get('NIMBUS_PORT', '5002')
NIMBUS_URL = f"http://{NIMBUS_HOST}:{NIMBUS_PORT}"

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get-random-number', methods=['GET'])
def get_random_number():

    try:
        # Make request to the nimbus service
        response = requests.get(f"{NIMBUS_URL}/calculate")
        if response.status_code == 200:
            calc = response.json()
            return jsonify({"number": calc["result"]})
        else:
            return jsonify({"error": f"Failed to get random number. Status code: {response.status_code}"}), 500
    except requests.exceptions.RequestException as e:
        return jsonify({"error": f"Failed to connect to nimbus service: {str(e)}"}), 500

@app.route('/health', methods=['GET'])
def health_check():
    """
    Simple health check endpoint
    """
    return jsonify({"status": "healthy"})

if __name__ == '__main__':
    print("Starting Nimbus WebUI service...")
    app.run(host='0.0.0.0', port=5000, debug=True)