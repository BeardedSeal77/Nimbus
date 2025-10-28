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

# Disable Flask/Werkzeug HTTP request logging (too noisy)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

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
    shared_state['autonomous_mode_trigger'] = False

    # AI configuration
    shared_state['global_get_dist'] = 1  # Depth detection enabled by default
    shared_state['depth_method'] = 'TRIG'  # TRIG method by default
    shared_state['global_target_distance'] = 0.0
    shared_state['global_intent'] = ''
    shared_state['global_command_status'] = 'none'
    shared_state['global_state_active'] = False

    # Object position calculation (polar to cartesian conversion)
    shared_state['calculate_position_trigger'] = False  # One-shot trigger
    shared_state['object_absolute_position'] = None  # Calculated world position
    shared_state['camera_config'] = None  # Camera FOV and resolution

    # Audio configuration
    shared_state['audio_recording'] = False
    shared_state['audio_process_trigger'] = False

    # Headset config
    shared_state['headset_yaw'] = 0.0
    shared_state['last_headset_update'] = 0.0

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

@app.route('/api/debug/ai_state', methods=['GET'])
def get_ai_state():
    """Return only relevant AI state for debugging"""
    if shared_state:
        return jsonify({
            'ai_results': shared_state.get('ai_results', {}),
        }), 200
    return {'status': 'error', 'message': 'Shared state not initialized'}, 500


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

        # Also update shared state for AI worker access
        if shared_state:
            shared_state['drone_telemetry'] = state
            # Store camera config if present
            if 'camera_config' in state and shared_state.get('camera_config') is None:
                shared_state['camera_config'] = state['camera_config']

        return {'status': 'ok'}, 200
    except Exception as e:
        logger.error(f"Update drone state error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/drone/video', methods=['POST'])
def update_drone_video():
    """Receive raw video frames from drone/simulation"""
    try:
        import base64
        video_data = request.get_json()

        jpeg_bytes = base64.b64decode(video_data['data'])

        with _topics_lock:
            _topics['drone/video']['data'] = {
                'jpeg': jpeg_bytes,
                'timestamp': video_data.get('timestamp', time.time()),
                'width': video_data.get('width'),
                'height': video_data.get('height')
            }
            _topics['drone/video']['timestamp'] = time.time()

        if shared_state:
            shared_state['raw_video_frame'] = jpeg_bytes

        return {'status': 'ok'}, 200
    except Exception as e:
        logger.error(f"Update drone video error: {e}")
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

@app.route('/navigation/target', methods=['GET'])
def get_navigation_target():
    """Return current navigation target from AI detection"""
    try:
        if shared_state and 'navigation_target' in shared_state:
            return {'target': shared_state['navigation_target']}, 200
        return {'target': None}, 200
    except Exception as e:
        logger.error(f"Get navigation target error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/navigation/object_position', methods=['GET'])
def get_object_position():
    """Return absolute object position in world coordinates"""
    try:
        if shared_state:
            obj_pos = shared_state.get('object_absolute_position')
            if obj_pos:
                return {
                    'object_position': obj_pos,
                    'has_position': True,
                    'intent': shared_state.get('global_intent'),
                    'object_name': shared_state.get('global_object')
                }, 200
        return {'object_position': None, 'has_position': False}, 200
    except Exception as e:
        logger.error(f"Get object position error: {e}")
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

@app.route('/api/autonomous_mode_trigger', methods=['GET'])
def get_autonomous_mode_trigger():
    """Check if autonomous mode should be enabled via voice command"""
    try:
        if shared_state:
            return {'trigger': shared_state.get('autonomous_mode_trigger', False)}, 200
        return {'trigger': False}, 200
    except Exception as e:
        logger.error(f"Get autonomous mode trigger error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/clear_autonomous_trigger', methods=['POST'])
def clear_autonomous_trigger():
    """Clear the autonomous mode trigger flag"""
    try:
        if shared_state:
            shared_state['autonomous_mode_trigger'] = False
            logger.info("Autonomous mode trigger cleared")
            return {'status': 'ok'}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Clear autonomous trigger error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

# ============================================================================
# AI CONTROL ENDPOINTS
# ============================================================================

@app.route('/api/target_status', methods=['GET'])
def get_target_status():
    target = shared_state.get('target_object')
    telemetry = shared_state.get('drone_telemetry', {})
    world_objects = telemetry.get('world_objects', {})

    if not target:
        return {"status": "error", "message": target}, 400
    
    mapping = {
        'car': 'car',
        'person': 'human',
        'bench': 'bench',
        'bottle': 'cardboard_box',
        'cabinet': 'cabinet'
    }
    webots_name = mapping.get(target.lower(), target.lower())

    obj = world_objects[webots_name]
    if not obj:
        return {"status": "error", "message": f"{target} not found"}, 404
    
    position = obj.get("position", {})
    detection_count = shared_state.get('detection_count')

    return {
        "status": "ok",
        "target": target,
        "detection_count": detection_count,
        "position": {
            "x": position.get("x", 0.0),
            "y": position.get("y", 0.0),
            "z": position.get("z", 0.0)
        }
    }

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
            shared_state['global_object'] = target  # Also set global_object for consistency
            logger.info(f"Target object set to: {target}")
            return {'status': 'ok', 'message': f'Target object set to {target}', 'target_object': target}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Set target error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/set_depth_method', methods=['POST'])
def set_depth_method():
    """Set depth detection method (SFM or TRIG)"""
    try:
        data = request.get_json()
        method = data.get('depth_method', 'TRIG')

        if method not in ['SFM', 'TRIG']:
            return {'status': 'error', 'message': 'Invalid method. Must be SFM or TRIG'}, 400

        if shared_state:
            shared_state['depth_method'] = method
            logger.info(f"Depth method set to: {method}")
            return {'status': 'ok', 'message': f'Depth method set to {method}', 'depth_method': method}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Set depth method error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/trigger_depth', methods=['POST'])
def trigger_depth():
    """Enable depth detection (TRIG runs continuously, SFM collects frames)"""
    try:
        if shared_state:
            shared_state['global_get_dist'] = 1
            method = shared_state.get('depth_method', 'TRIG')
            logger.info(f"Depth detection enabled with method: {method}")
            return {'status': 'ok', 'message': f'Depth detection enabled ({method} method)'}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Trigger depth error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/stop_depth', methods=['POST'])
def stop_depth():
    """Disable depth detection"""
    try:
        if shared_state:
            shared_state['global_get_dist'] = 0
            logger.info("Depth detection disabled")
            return {'status': 'ok', 'message': 'Depth detection disabled'}, 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Stop depth error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/audio/start', methods=['POST'])
def start_audio_recording():
    """Start audio recording from microphone"""
    try:
        if not ai_process or not ai_process.is_alive():
            return {'status': 'error', 'message': 'AI worker not running'}, 500

        shared_state['audio_recording'] = True
        logger.info("Audio recording started")
        return {'status': 'ok', 'message': 'Recording started'}, 200
    except Exception as e:
        logger.error(f"Start audio error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/audio/stop', methods=['POST'])
def stop_audio_recording():
    """Stop audio recording and process through STT + intent extraction"""
    try:
        if not ai_process or not ai_process.is_alive():
            return {'status': 'error', 'message': 'AI worker not running'}, 500

        shared_state['audio_recording'] = False
        shared_state['audio_process_trigger'] = True

        timeout = 10.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            if 'audio_result' in shared_state:
                result = dict(shared_state['audio_result'])
                del shared_state['audio_result']
                logger.info(f"Audio processing complete: {result}")
                return {'status': 'ok', 'result': result}, 200
            time.sleep(0.1)

        return {'status': 'error', 'message': 'Audio processing timeout'}, 500
    except Exception as e:
        logger.error(f"Stop audio error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/audio/process_text', methods=['POST'])
def process_text_command():
    """Process text command directly through intent extraction pipeline"""
    try:
        data = request.get_json()
        transcript = data.get('transcript', '').strip()

        if not transcript:
            return {'status': 'error', 'message': 'No transcript provided'}, 400

        if not ai_process or not ai_process.is_alive():
            return {'status': 'error', 'message': 'AI worker not running'}, 500

        shared_state['manual_transcript'] = transcript
        shared_state['manual_transcript_trigger'] = True

        timeout = 5.0
        start_time = time.time()

        while time.time() - start_time < timeout:
            if 'manual_transcript_result' in shared_state:
                result = dict(shared_state['manual_transcript_result'])
                del shared_state['manual_transcript_result']
                del shared_state['manual_transcript']
                shared_state['manual_transcript_trigger'] = False
                logger.info(f"Manual transcript processed: {result}")
                return {'status': 'ok', 'result': result}, 200
            time.sleep(0.1)

        return {'status': 'error', 'message': 'Processing timeout'}, 500
    except Exception as e:
        logger.error(f"Process text error: {e}")
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

@app.route('/api/debug/drone_telemetry', methods=['GET'])
def get_drone_telemetry_debug():
    """Debug endpoint to view full drone telemetry including world_objects"""
    try:
        if shared_state:
            drone_telemetry = shared_state.get('drone_telemetry', {})
            return jsonify(drone_telemetry), 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Debug telemetry error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/debug/position_calculation', methods=['GET'])
def get_position_calculation_debug():
    """Debug endpoint to view object position calculation state"""
    try:
        if shared_state:
            status = {
                'calculate_position_trigger': shared_state.get('calculate_position_trigger', False),
                'object_absolute_position': shared_state.get('object_absolute_position'),
                'camera_config': shared_state.get('camera_config'),
                'global_intent': shared_state.get('global_intent', ''),
                'global_object': shared_state.get('global_object', ''),
                'target_object': shared_state.get('target_object', ''),
                'has_camera_config': shared_state.get('camera_config') is not None,
                'has_drone_telemetry': shared_state.get('drone_telemetry') is not None
            }

            if shared_state.get('drone_telemetry'):
                telemetry = shared_state['drone_telemetry']
                status['drone_position'] = telemetry.get('position')
                status['drone_yaw'] = telemetry.get('orientation', {}).get('yaw')

            return jsonify(status), 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Debug position calculation error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/debug/depth_status', methods=['GET'])
def get_depth_status_debug():
    """Debug endpoint to view depth detection status and calculated distance"""
    try:
        if shared_state:
            # Get drone telemetry
            drone_telemetry = shared_state.get('drone_telemetry', {})
            drone_pos = drone_telemetry.get('position', {})
            world_objects = drone_telemetry.get('world_objects', {})

            # Get target object
            target_object = shared_state.get('global_object', 'car')

            # Simple mapping for webots name
            mapping = {'car': 'car', 'person': 'human', 'bench': 'bench', 'bottle': 'cardboard_box', 'cabinet': 'cabinet'}
            webots_name = mapping.get(target_object.lower(), target_object.lower())

            # Get object data
            object_data = world_objects.get(webots_name, {}) if webots_name else {}
            object_pos = object_data.get('position', {})

            # Calculate distance manually for verification
            calculated_distance = None
            distance_components = None
            if drone_pos and object_pos:
                try:
                    dx = float(object_pos.get('x', 0)) - float(drone_pos.get('x', 0))
                    dy = float(object_pos.get('y', 0)) - float(drone_pos.get('y', 0))
                    dz = float(object_pos.get('z', 0)) - float(drone_pos.get('z', 0))
                    calculated_distance = (dx**2 + dy**2 + dz**2) ** 0.5
                    distance_components = {'dx': dx, 'dy': dy, 'dz': dz}
                except Exception as calc_err:
                    distance_components = {'error': str(calc_err)}

            status = {
                'depth_enabled': shared_state.get('global_get_dist', 0) == 1,
                'depth_method': shared_state.get('depth_method', 'TRIG'),
                'target_object_yolo': target_object,
                'target_object_webots': webots_name,
                'target_distance_stored': shared_state.get('global_target_distance', 0.0),
                'calculated_distance_now': calculated_distance,
                'distance_components': distance_components,
                'depth_processing_busy': shared_state.get('depth_processing_busy', False),
                'object_detection_busy': shared_state.get('object_detection_busy', False),
                'successful_detection_counter': shared_state.get('successful_detection_counter', 0),
                'detection_count': shared_state.get('detection_count', 0),
                'drone_position': drone_pos,
                'object_position': object_pos,
                'object_type': object_data.get('type') if object_data else None,
                'available_objects': list(world_objects.keys()) if world_objects else [],
                'has_drone_telemetry': bool(drone_telemetry),
                'has_world_objects': bool(world_objects),
                'object_found_in_world': webots_name in world_objects if world_objects else False
            }
            return jsonify(status), 200
        return {'status': 'error', 'message': 'Shared state not initialized'}, 500
    except Exception as e:
        logger.error(f"Debug depth status error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

@app.route('/api/status', methods=['GET'])
def get_system_status():
    """Get overall system status (for web page)"""
    try:
        status = {
            'hub': {
                'active_topics': len(_topics),
                'sse_clients': len(_sse_clients)
            }
        }

        if shared_state:
            status['ai_worker'] = {
                'running': shared_state.get('ai_running', False),
                'heartbeat_age': time.time() - shared_state.get('last_heartbeat', 0),
                'processing_fps': shared_state.get('processing_fps', 0.0),
                'detection_count': shared_state.get('detection_count', 0)
            }

            # Include all global state variables for web interface
            status['global_object'] = shared_state.get('global_object', 'car')
            status['global_get_dist'] = shared_state.get('global_get_dist', 0)
            status['global_target_distance'] = shared_state.get('global_target_distance', 0.0)
            status['depth_processing_busy'] = shared_state.get('depth_processing_busy', False)
            status['depth_method'] = shared_state.get('depth_method', 'TRIG')  # SFM or TRIG
            status['depth_frame_count'] = shared_state.get('depth_frame_count', 5)
            status['reference_frame_position'] = shared_state.get('reference_frame_position', 10)
            status['frame_processing_interval_ms'] = shared_state.get('frame_processing_interval_ms', 100)
            status['ros2_host'] = shared_state.get('ros2_host', 'localhost')
            status['ros2_port'] = shared_state.get('ros2_port', 9090)
            status['global_intent'] = shared_state.get('global_intent', '')
            status['global_state_active'] = shared_state.get('global_state_active', False)
            status['global_command_status'] = shared_state.get('global_command_status', 'none')

        # Get drone telemetry
        with _topics_lock:
            drone_state = _topics.get('drone/state', {}).get('data')
            if drone_state:
                status['drone'] = {
                    'connected': drone_state.get('connected', False),
                    'mode': drone_state.get('mode', 'UNKNOWN')
                }

        return jsonify(status), 200
    except Exception as e:
        logger.error(f"System status error: {e}")
        return {'status': 'error', 'message': str(e)}, 500

# ============================================================================
# VIDEO STREAMING
# ============================================================================

@app.route('/video_feed')
def video_feed():
    """MJPEG video stream from AI processing"""
    def generate():
        import numpy as np
        import cv2

        # Create placeholder frame
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'Waiting for video...', (150, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        ret, placeholder_jpeg = cv2.imencode('.jpg', placeholder)
        placeholder_bytes = placeholder_jpeg.tobytes()

        while True:
            if shared_state and shared_state.get('video_frame'):
                frame = shared_state['video_frame']
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                # Send placeholder if no video yet
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + placeholder_bytes + b'\r\n')
            time.sleep(0.033)  # ~30 FPS

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/raw_video_feed')
def raw_video_feed():
    """MJPEG video stream of raw video from drone/simulation"""
    def generate():
        import numpy as np
        import cv2

        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'Waiting for raw video...', (150, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        ret, placeholder_jpeg = cv2.imencode('.jpg', placeholder)
        placeholder_bytes = placeholder_jpeg.tobytes()

        while True:
            if shared_state and shared_state.get('raw_video_frame'):
                frame = shared_state['raw_video_frame']
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + placeholder_bytes + b'\r\n')
            time.sleep(0.033)

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
# HEADSET ORIENTATION ENDPOINTS
# ============================================================================

@app.route('/api/headset/yaw', methods=['POST'])
def update_headset_yaw():
    """Receive headset yaw angle (from Unity)"""
    try:
        data = request.get_json()
        yaw = float(data.get('yaw', 0.0))
        
        # Update shared state
        if shared_state is not None:
            shared_state['headset_yaw'] = yaw
            shared_state['last_headset_update'] = time.time()

        # Also publish to broker for real-time consumers
        with _topics_lock:
            _topics['headset/yaw']['data'] = {'yaw': yaw}
            _topics['headset/yaw']['timestamp'] = time.time()
        _notify_sse_clients('headset/yaw', {'yaw': yaw})

        logger.info(f"Headset yaw updated: {yaw:.2f}")
        return {'status': 'ok', 'yaw': yaw}, 200

    except Exception as e:
        logger.error(f"Headset yaw update error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/api/headset/yaw', methods=['GET'])
def get_headset_yaw():
    """Return last known headset yaw"""
    try:
        if shared_state is None or 'headset_yaw' not in shared_state:
            return {'status': 'error', 'message': 'Headset yaw not available'}, 404

        yaw = shared_state.get('headset_yaw', 0.0)
        timestamp = shared_state.get('last_headset_update', 0)

        return {
            'status': 'ok',
            'yaw': yaw,
            'timestamp': timestamp
        }, 200

    except Exception as e:
        logger.error(f"Get headset yaw error: {e}")
        return {'status': 'error', 'message': str(e)}, 500
    
# ============================================================================
# MR JOYSTICK ENDPOINTS
# ============================================================================

@app.route('/api/mr/rotation', methods=['POST'])
def update_right_controller_yaw():
    """Receive right controller yaw from Unity"""
    try:
        data = request.get_json()
        yaw = float(data.get('yaw', 0.0))

        # Update shared state
        shared_state['right_controller_yaw'] = yaw

        return jsonify({'status': 'ok', 'yaw': yaw}), 200

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/mr/rotation', methods=['GET'])
def get_right_controller_yaw():
    """Return last known right controller yaw"""
    if 'right_controller_yaw' not in shared_state:
        return jsonify({'status': 'error', 'message': 'Yaw not available'}), 404

    yaw = shared_state['right_controller_yaw']
    return jsonify({'status': 'ok', 'yaw': yaw}), 200

@app.route('/api/mr/joystick', methods=['POST'])
def update_joystick_input():
    """Receive joystick pitch and roll from Unity"""
    try:
        data = request.get_json()

        pitch = float(data.get('pitch', 0.0))
        roll = float(data.get('roll', 0.0))

        # Update shared state
        shared_state['joystick_pitch'] = pitch
        shared_state['joystick_roll'] = roll

        return jsonify({'status': 'ok', 'pitch': pitch, 'roll': roll}), 200

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/mr/joystick', methods=['GET'])
def get_joystick_input():
    """Return last known joystick pitch and roll"""
    if 'joystick_pitch' not in shared_state or 'joystick_roll' not in shared_state:
        return jsonify({'status': 'error', 'message': 'Joystick data not available'}), 404

    pitch = shared_state['joystick_pitch']
    roll = shared_state['joystick_roll']

    return jsonify({'status': 'ok', 'pitch': pitch, 'roll': roll}), 200



# ============================================================================
# HUD MESSAGE ENDPOINTS
# ============================================================================

@app.route('/hud/message', methods=['POST'])
def set_hud_message():
    """Set or broadcast a short HUD message (for Unity display)"""
    try:
        data = request.get_json()
        message = data.get('message', '').strip()

        if not message:
            return {'status': 'error', 'message': 'Message cannot be empty'}, 400

        # Store in shared state (so it persists briefly)
        if shared_state is not None:
            shared_state['hud_message'] = {
                'text': message,
                'timestamp': time.time()
            }

        # Publish to message broker so subscribers can receive it
        with _topics_lock:
            _topics['hud/message']['data'] = {'message': message, 'timestamp': time.time()}
            _topics['hud/message']['timestamp'] = time.time()

        _notify_sse_clients('hud/message', {'message': message})

        logger.info(f"HUD message set: {message}")
        return {'status': 'ok', 'message': message}, 200

    except Exception as e:
        logger.error(f"HUD message error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


@app.route('/hud/message', methods=['GET'])
def get_hud_message():
    """Retrieve the latest HUD message (for Unity polling)"""
    try:
        if shared_state is None or 'hud_message' not in shared_state:
            return {'status': 'ok', 'message': None}, 200

        hud_data = shared_state.get('hud_message', {})
        return {
            'status': 'ok',
            'message': hud_data.get('text'),
            'timestamp': hud_data.get('timestamp')
        }, 200

    except Exception as e:
        logger.error(f"Get HUD message error: {e}")
        return {'status': 'error', 'message': str(e)}, 500


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

def init_signal_handlers():
    """Initialize signal handlers (only in main thread)"""
    try:
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
    except ValueError:
        # Not in main thread, skip signal registration
        pass

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
        # Initialize signal handlers
        init_signal_handlers()

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