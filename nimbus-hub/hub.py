"""
Nimbus Central Hub
Unified Flask server that:
- Hosts web interface
- Message broker (pub/sub)
- Spawns nimbus-ai worker process
- Manages shared state
"""

from flask import Flask, request, jsonify, Response, render_template
import json
import threading
import time
from collections import defaultdict
import logging
import os
import sys
from multiprocessing import Process, Manager
import signal

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Create Flask app
app = Flask(__name__)

# ============================================================================
# SHARED STATE FOR MULTIPROCESSING
# ============================================================================

# Global manager and shared state (initialized later)
manager = None
shared_state = None
ai_process = None

def init_shared_state():
    """Initialize shared state dictionary"""
    global manager, shared_state
    manager = Manager()
    shared_state = manager.dict()

    # Initialize state structure
    shared_state['cancel'] = False
    shared_state['mode'] = 'MANUAL'
    shared_state['target_object'] = 'car'
    shared_state['ai_running'] = True
    shared_state['last_heartbeat'] = time.time()
    shared_state['ai_results'] = {}
    shared_state['video_frame'] = None
    shared_state['processing_fps'] = 0.0
    shared_state['detection_count'] = 0

    # AI configuration
    shared_state['global_get_dist'] = 0
    shared_state['global_target_distance'] = 0.0
    shared_state['global_intent'] = ''
    shared_state['global_command_status'] = 'none'
    shared_state['global_state_active'] = False

    logger.info("Shared state initialized")

# ============================================================================
# MESSAGE BROKER - TOPIC PUB/SUB
# ============================================================================

# Latest messages by topic
_topics = defaultdict(lambda: {'data': None, 'timestamp': 0, 'subscribers': []})
_topics_lock = threading.Lock()

# SSE connections for real-time streaming
_sse_clients = []
_sse_lock = threading.Lock()

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
# DRONE TELEMETRY ENDPOINTS
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

# ============================================================================
# AI CONTROL ENDPOINTS
# ============================================================================

@app.route('/api/cancel', methods=['POST'])
def cancel_processing():
    """Cancel AI processing"""
    try:
        if shared_state:
            shared_state['cancel'] = True
            logger.info("AI processing cancel requested")
            return {'status': 'ok', 'message': 'Cancel signal sent'}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Cancel error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/set_target_object', methods=['POST'])
def set_target_object():
    """Set target object for AI detection"""
    try:
        data = request.get_json()
        target = data.get('object', 'car')
        if shared_state:
            shared_state['target_object'] = target
            logger.info(f"Target object set to: {target}")
            return {'status': 'ok', 'target_object': target}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Set target error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/ai_status', methods=['GET'])
def get_ai_status():
    """Get AI worker status"""
    try:
        if shared_state:
            status = {
                'ai_running': shared_state.get('ai_running', False),
                'last_heartbeat': shared_state.get('last_heartbeat', 0),
                'heartbeat_age': time.time() - shared_state.get('last_heartbeat', 0),
                'processing_fps': shared_state.get('processing_fps', 0.0),
                'detection_count': shared_state.get('detection_count', 0),
                'target_object': shared_state.get('target_object', 'unknown'),
                'mode': shared_state.get('mode', 'MANUAL')
            }
            return jsonify(status), 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"AI status error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

# ============================================================================
# VIDEO STREAMING
# ============================================================================

@app.route('/video_feed')
def video_feed():
    """MJPEG video stream from AI processing"""
    def generate():
        while True:
            if shared_state and shared_state.get('video_frame'):
                frame = shared_state['video_frame']
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033)  # ~30 FPS

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

# ============================================================================
# WEB INTERFACE ROUTES
# ============================================================================

# Import and register web routes blueprint
try:
    from routes.web_routes import web_bp
    app.register_blueprint(web_bp)
    logger.info("Web routes blueprint registered")
except ImportError as e:
    logger.error(f"Failed to import web routes: {e}")

# ============================================================================
# HEALTH CHECK
# ============================================================================

@app.route('/health')
def health():
    """Health check"""
    status = {
        'status': 'healthy',
        'active_topics': len(_topics),
        'sse_clients': len(_sse_clients)
    }

    if shared_state:
        status['ai_worker'] = {
            'running': shared_state.get('ai_running', False),
            'heartbeat_age': time.time() - shared_state.get('last_heartbeat', 0)
        }

    return jsonify(status), 200

# ============================================================================
# AI WORKER PROCESS MANAGEMENT
# ============================================================================

def start_ai_worker():
    """Start the AI worker process"""
    global ai_process

    try:
        # Add nimbus-ai to Python path
        nimbus_ai_path = os.path.join(os.path.dirname(__file__), '..', 'nimbus-ai')
        nimbus_ai_path = os.path.abspath(nimbus_ai_path)

        if nimbus_ai_path not in sys.path:
            sys.path.insert(0, nimbus_ai_path)

        # Import and start AI worker
        from app import run_ai_worker

        ai_process = Process(target=run_ai_worker, args=(shared_state,), daemon=True)
        ai_process.start()

        logger.info(f"AI worker process started (PID: {ai_process.pid})")

    except Exception as e:
        logger.error(f"Failed to start AI worker: {e}")
        logger.error(f"Make sure nimbus-ai/app.py has run_ai_worker() function")

def stop_ai_worker():
    """Stop the AI worker process"""
    global ai_process

    if ai_process and ai_process.is_alive():
        logger.info("Stopping AI worker process...")
        if shared_state:
            shared_state['ai_running'] = False
        ai_process.terminate()
        ai_process.join(timeout=5)
        logger.info("AI worker process stopped")

def heartbeat_monitor():
    """Monitor AI worker heartbeat"""
    while True:
        try:
            if shared_state:
                heartbeat_age = time.time() - shared_state.get('last_heartbeat', 0)
                if heartbeat_age > 10:
                    logger.warning(f"AI worker heartbeat stale ({heartbeat_age:.1f}s)")
                    if heartbeat_age > 30:
                        logger.error("AI worker appears hung, consider restarting")
        except Exception as e:
            logger.error(f"Heartbeat monitor error: {e}")

        time.sleep(5)

# ============================================================================
# SIGNAL HANDLERS
# ============================================================================

def signal_handler(sig, frame):
    """Handle shutdown signals"""
    logger.info("Shutdown signal received")
    stop_ai_worker()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ============================================================================
# STARTUP
# ============================================================================

if __name__ == '__main__':
    logger.info("=" * 60)
    logger.info("NIMBUS CENTRAL HUB")
    logger.info("=" * 60)
    logger.info("Unified Flask server with AI worker process")
    logger.info("Starting on http://localhost:5000")
    logger.info("")
    logger.info("Message Broker Endpoints:")
    logger.info("  POST   /publish              - Publish to any topic")
    logger.info("  GET    /get/<topic>          - Get latest from topic")
    logger.info("  GET    /topics               - List all topics")
    logger.info("  GET    /stream/<topic>       - SSE stream for topic")
    logger.info("")
    logger.info("Drone Endpoints:")
    logger.info("  POST   /drone/state          - Update drone state")
    logger.info("  GET    /drone/state          - Get drone state")
    logger.info("  POST   /control/command      - Send control command")
    logger.info("")
    logger.info("AI Control:")
    logger.info("  POST   /api/cancel           - Cancel AI processing")
    logger.info("  POST   /api/set_target_object - Set target object")
    logger.info("  GET    /api/ai_status        - Get AI worker status")
    logger.info("")
    logger.info("Web Interface:")
    logger.info("  GET    /                     - Main control interface")
    logger.info("  GET    /video_feed           - MJPEG video stream")
    logger.info("  GET    /health               - Health check")
    logger.info("=" * 60)

    try:
        # Initialize shared state
        init_shared_state()

        # Start heartbeat monitor thread
        monitor_thread = threading.Thread(target=heartbeat_monitor, daemon=True)
        monitor_thread.start()
        logger.info("Heartbeat monitor started")

        # Start AI worker process
        start_ai_worker()

        # Start Flask server
        app.run(
            host='0.0.0.0',
            port=5000,
            debug=False,
            threaded=True,
            use_reloader=False  # Prevent process duplication
        )

    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
    except Exception as e:
        logger.error(f"Application error: {e}")
    finally:
        stop_ai_worker()
        logger.info("Nimbus Hub shutdown complete")