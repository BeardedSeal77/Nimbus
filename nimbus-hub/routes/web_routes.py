"""
Web Routes
HTML interface and web-based control endpoints
"""

from flask import Blueprint, render_template, current_app, Response, stream_with_context, request
import json
import time

web_bp = Blueprint('web', __name__)

# Global drone state (will be updated by services)
_drone_state = {
    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'angular_velocity': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    'facing_vector': {'x': 1.0, 'y': 0.0, 'z': 0.0},
    'mode': 'MANUAL',
    'target_altitude': 2.0,
    'distance_to_home': 0.0,
    'connected': False
}

def update_drone_state(state_dict):
    """Update global drone state (called by services)"""
    global _drone_state
    _drone_state.update(state_dict)

@web_bp.route('/')
def index():
    """Main control interface"""
    return render_template('control.html')

@web_bp.route('/status')
def status_page():
    """System status page"""
    return render_template('status.html')

@web_bp.route('/docs')
def documentation():
    """Documentation page"""
    return render_template('docs.html')

@web_bp.route('/health')
def health_check():
    """Simple health check endpoint"""
    return {
        'status': 'healthy',
        'services': {
            'ai_service': current_app.config.get('AI_SERVICE_RUNNING', False),
            'ros2_service': current_app.config.get('ROS2_SERVICE_RUNNING', False),
            'display_service': current_app.config.get('DISPLAY_SERVICE_RUNNING', False)
        }
    }

@web_bp.route('/drone_state_stream')
def drone_state_stream():
    """Server-Sent Events stream for live drone state updates"""
    def generate():
        import requests

        # Create persistent session for this SSE client to reuse connections
        session = requests.Session()
        adapter = requests.adapters.HTTPAdapter(
            pool_connections=2,
            pool_maxsize=5
        )
        session.mount('http://', adapter)

        try:
            while True:
                try:
                    # Get latest drone state via HTTP GET using persistent session
                    response = session.get('http://127.0.0.1:5000/drone/state', timeout=0.05)
                    if response.status_code == 200:
                        hub_state = response.json()
                        _drone_state.update(hub_state)
                except:
                    pass  # Keep last known state

                data = json.dumps(_drone_state)
                yield f"data: {data}\n\n"
                time.sleep(0.01)  # Update at 100Hz
        finally:
            # Cleanup session when client disconnects
            session.close()

    return Response(
        stream_with_context(generate()),
        mimetype='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'X-Accel-Buffering': 'no'
        }
    )

@web_bp.route('/update_drone_state', methods=['POST'])
def update_drone_state_http():
    """Receive drone state updates directly from Webots via HTTP POST"""
    try:
        state = request.get_json()
        update_drone_state(state)
        return {'status': 'ok'}, 200
    except Exception as e:
        return {'status': 'error', 'message': str(e)}, 500