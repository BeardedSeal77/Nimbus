"""
Nimbus Central Communication Hub
Fast message broker for real-time communication between:
- Webots (simulation)
- nimbus-ai (vision/ML)
- nimbus-robotics (PID/control)
- Web interface (monitoring)

Uses HTTP for simplicity and speed (no Docker/ROS2 overhead)
"""

from flask import Flask, request, jsonify, Response
import json
import threading
import time
from collections import defaultdict
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)

# ============================================================================
# GLOBAL STATE STORE
# ============================================================================

# Latest messages by topic
_topics = defaultdict(lambda: {'data': None, 'timestamp': 0, 'subscribers': []})
_topics_lock = threading.Lock()

# SSE connections for real-time streaming
_sse_clients = []
_sse_lock = threading.Lock()


# ============================================================================
# CORE API - PUBLISH/SUBSCRIBE
# ============================================================================

@app.route('/publish', methods=['POST'])
def publish():
    """Publish data to a topic"""
    try:
        data = request.get_json()
        topic = data.get('topic')
        message = data.get('message')

        if not topic or message is None:
            return {'status': 'error', 'message': 'Missing topic or message'}, 400

        with _topics_lock:
            _topics[topic]['data'] = message
            _topics[topic]['timestamp'] = time.time()

        # Notify SSE clients
        _notify_sse_clients(topic, message)

        return {'status': 'ok'}, 200
    except Exception as e:
        logger.error(f"Publish error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/get/<topic>', methods=['GET'])
def get_topic(topic):
    """Get latest data from a topic"""
    try:
        with _topics_lock:
            data = _topics[topic]['data']
            timestamp = _topics[topic]['timestamp']

        if data is None:
            return {'status': 'error', 'message': 'Topic not found'}, 404

        return {
            'status': 'ok',
            'topic': topic,
            'message': data,
            'timestamp': timestamp
        }, 200
    except Exception as e:
        logger.error(f"Get topic error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/topics', methods=['GET'])
def list_topics():
    """List all active topics"""
    try:
        with _topics_lock:
            topics_list = [
                {
                    'topic': topic,
                    'timestamp': info['timestamp'],
                    'has_data': info['data'] is not None
                }
                for topic, info in _topics.items()
            ]

        return {
            'status': 'ok',
            'topics': topics_list
        }, 200
    except Exception as e:
        logger.error(f"List topics error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


# ============================================================================
# SERVER-SENT EVENTS FOR REAL-TIME STREAMING
# ============================================================================

@app.route('/stream/<topic>')
def stream_topic(topic):
    """Server-Sent Events stream for a specific topic"""
    def generate():
        # Add this client to the list
        client_id = id(generate)
        with _sse_lock:
            _sse_clients.append({'id': client_id, 'topic': topic, 'queue': []})

        logger.info(f"SSE client connected to topic: {topic}")

        try:
            while True:
                # Check for new messages
                with _sse_lock:
                    client = next((c for c in _sse_clients if c['id'] == client_id), None)
                    if client and client['queue']:
                        message = client['queue'].pop(0)
                        yield f"data: {json.dumps(message)}\n\n"
                    else:
                        # Send keepalive
                        yield f": keepalive\n\n"

                time.sleep(0.01)  # 100Hz polling
        except GeneratorExit:
            # Client disconnected
            with _sse_lock:
                _sse_clients[:] = [c for c in _sse_clients if c['id'] != client_id]
            logger.info(f"SSE client disconnected from topic: {topic}")

    return Response(generate(), mimetype='text/event-stream')


def _notify_sse_clients(topic, message):
    """Notify all SSE clients subscribed to a topic"""
    with _sse_lock:
        for client in _sse_clients:
            if client['topic'] == topic or client['topic'] == '*':
                client['queue'].append({'topic': topic, 'message': message, 'timestamp': time.time()})


# ============================================================================
# CONVENIENCE ENDPOINTS FOR SPECIFIC DATA TYPES
# ============================================================================

@app.route('/drone/state', methods=['POST'])
def update_drone_state():
    """Shortcut for publishing drone state"""
    try:
        state = request.get_json()

        with _topics_lock:
            _topics['drone/state']['data'] = state
            _topics['drone/state']['timestamp'] = time.time()

        _notify_sse_clients('drone/state', state)

        return {'status': 'ok'}, 200
    except Exception as e:
        logger.error(f"Update drone state error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/drone/state', methods=['GET'])
def get_drone_state():
    """Shortcut for getting drone state"""
    try:
        with _topics_lock:
            state = _topics['drone/state']['data']

        if state is None:
            return {'status': 'error', 'message': 'No drone state available'}, 404

        return state, 200
    except Exception as e:
        logger.error(f"Get drone state error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/control/command', methods=['POST'])
def send_control_command():
    """Shortcut for publishing control commands"""
    try:
        command = request.get_json()

        with _topics_lock:
            _topics['control/command']['data'] = command
            _topics['control/command']['timestamp'] = time.time()

        _notify_sse_clients('control/command', command)

        return {'status': 'ok'}, 200
    except Exception as e:
        logger.error(f"Send control command error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/health')
def health():
    """Health check"""
    return {
        'status': 'healthy',
        'active_topics': len(_topics),
        'sse_clients': len(_sse_clients)
    }, 200


# ============================================================================
# STARTUP
# ============================================================================

if __name__ == '__main__':
    logger.info("=" * 60)
    logger.info("NIMBUS CENTRAL COMMUNICATION HUB")
    logger.info("=" * 60)
    logger.info("Starting on http://localhost:8000")
    logger.info("")
    logger.info("Endpoints:")
    logger.info("  POST   /publish              - Publish to any topic")
    logger.info("  GET    /get/<topic>          - Get latest from topic")
    logger.info("  GET    /topics               - List all topics")
    logger.info("  GET    /stream/<topic>       - SSE stream for topic")
    logger.info("  POST   /drone/state          - Update drone state")
    logger.info("  GET    /drone/state          - Get drone state")
    logger.info("  POST   /control/command      - Send control command")
    logger.info("  GET    /health               - Health check")
    logger.info("=" * 60)

    app.run(
        host='0.0.0.0',
        port=8000,
        debug=False,
        threaded=True
    )