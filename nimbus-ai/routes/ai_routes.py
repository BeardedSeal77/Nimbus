"""
AI Control Routes
API endpoints for controlling AI processing and depth detection
"""

from flask import Blueprint, jsonify, request, current_app
import logging

logger = logging.getLogger(__name__)
ai_bp = Blueprint('ai', __name__)

@ai_bp.route('/status')
def get_status():
    """Get current AI system status"""
    with current_app.config['STATE_LOCK']:
        status = {
            'global_object': current_app.config['GLOBAL_OBJECT'],
            'global_get_dist': current_app.config['GLOBAL_GET_DIST'],
            'global_target_distance': current_app.config['GLOBAL_TARGET_DISTANCE'],
            'global_intent': current_app.config['GLOBAL_INTENT'],
            'object_detection_busy': current_app.config['OBJECT_DETECTION_BUSY'],
            'depth_processing_busy': current_app.config['DEPTH_PROCESSING_BUSY'],
            'depth_frame_count': current_app.config['DEPTH_FRAME_COUNT'],
            'reference_frame_position': current_app.config['REFERENCE_FRAME_POSITION'],
            'frame_processing_interval_ms': current_app.config['FRAME_PROCESSING_INTERVAL_MS'],
            'global_state_active': current_app.config['GLOBAL_STATE_ACTIVE'],
            'global_command_status': current_app.config['GLOBAL_COMMAND_STATUS'],
            'ros2_host': current_app.config['ROS2_HOST'],
            'ros2_port': current_app.config['ROS2_PORT'],
            'services': {
                'ai_service_running': current_app.config['AI_SERVICE_RUNNING'],
                'ros2_service_running': current_app.config['ROS2_SERVICE_RUNNING'],
                'display_service_running': current_app.config['DISPLAY_SERVICE_RUNNING']
            }
        }
    
    # Get additional service statistics
    try:
        from services.ros2_service import get_connection_status
        from services.display_service import get_display_stats
        
        status['ros2_connection'] = get_connection_status()
        status['display_stats'] = get_display_stats()
    except Exception as e:
        logger.warning(f"Could not get service stats: {e}")
    
    return jsonify(status)

@ai_bp.route('/trigger_depth', methods=['POST'])
def trigger_depth_detection():
    """Trigger depth detection by setting GLOBAL_GET_DIST = 1 and resetting counters"""
    try:
        with current_app.config['STATE_LOCK']:
            current_app.config['GLOBAL_GET_DIST'] = 1
            current_app.config['GLOBAL_TARGET_DISTANCE'] = 0.0  # Reset previous distance
        
        # Reset AI service depth collection counters
        try:
            from services.ai_service import reset_depth_collection
            reset_depth_collection()
        except Exception as e:
            logger.warning(f"Could not reset depth collection: {e}")
            
        logger.info("Depth detection triggered via API")
        return jsonify({
            'success': True,
            'message': 'Depth detection triggered - collecting frames from object detections',
            'global_get_dist': current_app.config['GLOBAL_GET_DIST']
        })
    except Exception as e:
        logger.error(f"Error triggering depth detection: {e}")
        return jsonify({
            'success': False,
            'message': f'Error triggering depth detection: {e}'
        }), 500

@ai_bp.route('/stop_depth', methods=['POST'])
def stop_depth_detection():
    """Stop depth detection by setting GLOBAL_GET_DIST = 0"""
    try:
        with current_app.config['STATE_LOCK']:
            current_app.config['GLOBAL_GET_DIST'] = 0
            
        logger.info("Depth detection stopped via API")
        return jsonify({
            'success': True,
            'message': 'Depth detection stopped',
            'global_get_dist': current_app.config['GLOBAL_GET_DIST']
        })
    except Exception as e:
        logger.error(f"Error stopping depth detection: {e}")
        return jsonify({
            'success': False,
            'message': f'Error stopping depth detection: {e}'
        }), 500

@ai_bp.route('/set_target_object', methods=['POST'])
def set_target_object():
    """Set the target object for detection"""
    try:
        data = request.get_json()
        target_object = data.get('object', 'Person')
        
        with current_app.config['STATE_LOCK']:
            current_app.config['GLOBAL_OBJECT'] = target_object
            
        logger.info(f"Target object set to: {target_object}")
        return jsonify({
            'success': True,
            'message': f'Target object set to: {target_object}',
            'global_object': current_app.config['GLOBAL_OBJECT']
        })
    except Exception as e:
        logger.error(f"Error setting target object: {e}")
        return jsonify({
            'success': False,
            'message': f'Error setting target object: {e}'
        }), 500

@ai_bp.route('/reset_system', methods=['POST'])
def reset_system():
    """Reset the AI system parameters"""
    try:
        with current_app.config['STATE_LOCK']:
            current_app.config['GLOBAL_GET_DIST'] = 0
            current_app.config['GLOBAL_TARGET_DISTANCE'] = 0.0
            current_app.config['GLOBAL_INTENT'] = ""
            
        logger.info("System reset via API")
        return jsonify({
            'success': True,
            'message': 'System reset successfully',
            'status': {
                'global_get_dist': current_app.config['GLOBAL_GET_DIST'],
                'global_target_distance': current_app.config['GLOBAL_TARGET_DISTANCE'],
                'global_intent': current_app.config['GLOBAL_INTENT']
            }
        })
    except Exception as e:
        logger.error(f"Error resetting system: {e}")
        return jsonify({
            'success': False,
            'message': f'Error resetting system: {e}'
        }), 500

@ai_bp.route('/set_depth_params', methods=['POST'])
def set_depth_parameters():
    """Set depth detection parameters"""
    try:
        data = request.get_json()
        frame_count = data.get('frame_count', current_app.config['DEPTH_FRAME_COUNT'])
        reference_position = data.get('reference_frame_position', current_app.config['REFERENCE_FRAME_POSITION'])
        
        with current_app.config['STATE_LOCK']:
            current_app.config['DEPTH_FRAME_COUNT'] = int(frame_count)
            current_app.config['REFERENCE_FRAME_POSITION'] = int(reference_position)
            
        logger.info(f"Depth parameters updated: count={frame_count}, reference_position={reference_position}")
        return jsonify({
            'success': True,
            'message': 'Depth parameters updated',
            'depth_frame_count': current_app.config['DEPTH_FRAME_COUNT'],
            'reference_frame_position': current_app.config['REFERENCE_FRAME_POSITION']
        })
    except Exception as e:
        logger.error(f"Error setting depth parameters: {e}")
        return jsonify({
            'success': False,
            'message': f'Error setting depth parameters: {e}'
        }), 500

@ai_bp.route('/set_intent', methods=['POST'])
def set_intent():
    """Set the current flight intent"""
    try:
        data = request.get_json()
        intent = data.get('intent', '')
        
        with current_app.config['STATE_LOCK']:
            current_app.config['GLOBAL_INTENT'] = intent
            
        logger.info(f"Intent set to: {intent}")
        return jsonify({
            'success': True,
            'message': f'Intent set to: {intent}',
            'global_intent': current_app.config['GLOBAL_INTENT']
        })
    except Exception as e:
        logger.error(f"Error setting intent: {e}")
        return jsonify({
            'success': False,
            'message': f'Error setting intent: {e}'
        }), 500

@ai_bp.route('/detection_result')
def get_detection_result():
    """Get the latest detection result"""
    try:
        from services.ai_service import get_latest_detection_result
        result = get_latest_detection_result()
        return jsonify(result)
    except Exception as e:
        logger.error(f"Error getting detection result: {e}")
        return jsonify({
            'error': f'Error getting detection result: {e}'
        }), 500

@ai_bp.route('/service_stats')
def get_service_stats():
    """Get detailed service statistics"""
    try:
        stats = {}
        
        # ROS2 service stats
        try:
            from services.ros2_service import get_connection_status
            stats['ros2'] = get_connection_status()
        except Exception as e:
            stats['ros2'] = {'error': str(e)}
        
        # Display service stats
        try:
            from services.display_service import get_display_stats
            stats['display'] = get_display_stats()
        except Exception as e:
            stats['display'] = {'error': str(e)}
        
        # AI service stats
        stats['ai'] = {
            'service_running': current_app.config['AI_SERVICE_RUNNING']
        }
        
        return jsonify(stats)
    except Exception as e:
        logger.error(f"Error getting service stats: {e}")
        return jsonify({
            'error': f'Error getting service stats: {e}'
        }), 500

@ai_bp.route('/set_ai_params', methods=['POST'])
def set_ai_parameters():
    """Set AI processing parameters"""
    try:
        data = request.get_json()
        frame_interval_ms = data.get('frame_processing_interval_ms', current_app.config['FRAME_PROCESSING_INTERVAL_MS'])
        
        with current_app.config['STATE_LOCK']:
            current_app.config['FRAME_PROCESSING_INTERVAL_MS'] = int(frame_interval_ms)
            
        logger.info(f"AI parameters updated: frame_interval={frame_interval_ms}ms")
        return jsonify({
            'success': True,
            'message': f'AI processing interval set to {frame_interval_ms}ms',
            'frame_processing_interval_ms': current_app.config['FRAME_PROCESSING_INTERVAL_MS']
        })
    except Exception as e:
        logger.error(f"Error setting AI parameters: {e}")
        return jsonify({
            'success': False,
            'message': f'Error setting AI parameters: {e}'
        }), 500

@ai_bp.route('/set_ros2_params', methods=['POST'])
def set_ros2_parameters():
    """Set ROS2 connection parameters (requires restart to take effect)"""
    try:
        data = request.get_json()
        ros2_host = data.get('ros2_host', current_app.config['ROS2_HOST'])
        ros2_port = data.get('ros2_port', current_app.config['ROS2_PORT'])
        
        with current_app.config['STATE_LOCK']:
            current_app.config['ROS2_HOST'] = ros2_host
            current_app.config['ROS2_PORT'] = int(ros2_port)
            
        logger.info(f"ROS2 parameters updated: host={ros2_host}, port={ros2_port}")
        return jsonify({
            'success': True,
            'message': f'ROS2 parameters updated to {ros2_host}:{ros2_port}',
            'ros2_host': current_app.config['ROS2_HOST'],
            'ros2_port': current_app.config['ROS2_PORT']
        })
    except Exception as e:
        logger.error(f"Error setting ROS2 parameters: {e}")
        return jsonify({
            'success': False,
            'message': f'Error setting ROS2 parameters: {e}'
        }), 500

@ai_bp.route('/set_global_state', methods=['POST'])
def set_global_state_params():
    """Set global state parameters"""
    try:
        data = request.get_json()
        global_intent = data.get('global_intent', current_app.config['GLOBAL_INTENT'])
        global_state_active = data.get('global_state_active', current_app.config['GLOBAL_STATE_ACTIVE'])
        global_command_status = data.get('global_command_status', current_app.config['GLOBAL_COMMAND_STATUS'])
        
        with current_app.config['STATE_LOCK']:
            current_app.config['GLOBAL_INTENT'] = global_intent
            current_app.config['GLOBAL_STATE_ACTIVE'] = bool(global_state_active)
            current_app.config['GLOBAL_COMMAND_STATUS'] = global_command_status
            
        logger.info(f"Global state updated: intent='{global_intent}', active={global_state_active}, status={global_command_status}")
        return jsonify({
            'success': True,
            'message': 'Global state parameters updated',
            'global_intent': current_app.config['GLOBAL_INTENT'],
            'global_state_active': current_app.config['GLOBAL_STATE_ACTIVE'],
            'global_command_status': current_app.config['GLOBAL_COMMAND_STATUS']
        })
    except Exception as e:
        logger.error(f"Error setting global state: {e}")
        return jsonify({
            'success': False,
            'message': f'Error setting global state: {e}'
        }), 500