"""
Web Routes
HTML interface and web-based control endpoints
"""

from flask import Blueprint, render_template, current_app

web_bp = Blueprint('web', __name__)

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