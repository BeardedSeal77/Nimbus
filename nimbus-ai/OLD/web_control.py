#!/usr/bin/env python3
"""
Web Control Server for Nimbus AI System
Flask server to control depth detection and other AI parameters
"""

from flask import Flask, render_template, jsonify, request
import threading
import time
import sys
import os

# Add project paths
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

# Import AI module to access global variables
import AI

app = Flask(__name__)

# Thread lock for safe global variable access
control_lock = threading.Lock()

@app.route('/')
def index():
    """Main control interface"""
    return render_template('control.html')

@app.route('/api/status')
def get_status():
    """Get current AI system status"""
    with control_lock:
        status = {
            'global_object': AI.GLOBAL_OBJECT,
            'global_get_dist': AI.GLOBAL_GET_DIST,
            'global_target_distance': AI.GLOBAL_TARGET_DISTANCE,
            'global_intent': AI.GLOBAL_INTENT,
            'object_detection_busy': AI.OBJECT_DETECTION_BUSY,
            'depth_processing_busy': AI.DEPTH_PROCESSING_BUSY,
            'depth_frame_interval': AI.DEPTH_FRAME_INTERVAL,
            'depth_frame_count': AI.DEPTH_FRAME_COUNT
        }
    return jsonify(status)

@app.route('/api/trigger_depth', methods=['POST'])
def trigger_depth_detection():
    """Trigger depth detection by setting GLOBAL_GET_DIST = 1"""
    try:
        with control_lock:
            AI.GLOBAL_GET_DIST = 1
            AI.GLOBAL_TARGET_DISTANCE = 0.0  # Reset previous distance
            
        return jsonify({
            'success': True,
            'message': 'Depth detection triggered',
            'global_get_dist': AI.GLOBAL_GET_DIST
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error triggering depth detection: {e}'
        }), 500

@app.route('/api/stop_depth', methods=['POST'])
def stop_depth_detection():
    """Stop depth detection by setting GLOBAL_GET_DIST = 0"""
    try:
        with control_lock:
            AI.GLOBAL_GET_DIST = 0
            
        return jsonify({
            'success': True,
            'message': 'Depth detection stopped',
            'global_get_dist': AI.GLOBAL_GET_DIST
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error stopping depth detection: {e}'
        }), 500

@app.route('/api/set_target_object', methods=['POST'])
def set_target_object():
    """Set the target object for detection"""
    try:
        data = request.get_json()
        target_object = data.get('object', 'Person')
        
        with control_lock:
            AI.GLOBAL_OBJECT = target_object
            
        return jsonify({
            'success': True,
            'message': f'Target object set to: {target_object}',
            'global_object': AI.GLOBAL_OBJECT
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error setting target object: {e}'
        }), 500

@app.route('/api/reset_system', methods=['POST'])
def reset_system():
    """Reset the AI system parameters"""
    try:
        with control_lock:
            AI.GLOBAL_GET_DIST = 0
            AI.GLOBAL_TARGET_DISTANCE = 0.0
            AI.GLOBAL_INTENT = ""
            
        return jsonify({
            'success': True,
            'message': 'System reset successfully',
            'status': {
                'global_get_dist': AI.GLOBAL_GET_DIST,
                'global_target_distance': AI.GLOBAL_TARGET_DISTANCE,
                'global_intent': AI.GLOBAL_INTENT
            }
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error resetting system: {e}'
        }), 500

@app.route('/api/set_depth_params', methods=['POST'])
def set_depth_parameters():
    """Set depth detection parameters"""
    try:
        data = request.get_json()
        frame_interval = data.get('frame_interval', AI.DEPTH_FRAME_INTERVAL)
        frame_count = data.get('frame_count', AI.DEPTH_FRAME_COUNT)
        
        with control_lock:
            AI.DEPTH_FRAME_INTERVAL = int(frame_interval)
            AI.DEPTH_FRAME_COUNT = int(frame_count)
            
        return jsonify({
            'success': True,
            'message': f'Depth parameters updated',
            'depth_frame_interval': AI.DEPTH_FRAME_INTERVAL,
            'depth_frame_count': AI.DEPTH_FRAME_COUNT
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'Error setting depth parameters: {e}'
        }), 500

def run_web_server(host='localhost', port=5000, debug=False):
    """Run the Flask web server"""
    print(f"Starting web control server at http://{host}:{port}")
    print("Available endpoints:")
    print("   GET  /                  - Main control interface")
    print("   GET  /api/status        - Get system status")
    print("   POST /api/trigger_depth - Trigger depth detection")
    print("   POST /api/stop_depth    - Stop depth detection")
    print("   POST /api/set_target_object - Set target object")
    print("   POST /api/reset_system  - Reset system parameters")
    print("   POST /api/set_depth_params - Set depth parameters")
    print("")
    
    app.run(host=host, port=port, debug=debug, threaded=True)

if __name__ == '__main__':
    # Run the web server standalone
    run_web_server(debug=True)