#!/usr/bin/env python3
"""
Nimbus AI Main Application
Flask-based web application with AI processing services
"""

import os
import sys
import threading
import logging
from flask import Flask

# Add project paths
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Create Flask app
app = Flask(__name__)

# Global configuration and state variables
app.config.update({
    # AI Processing Configuration
    'FRAME_PROCESSING_INTERVAL_MS': 100,
    'ACTIVATION_PHRASES': ["ok drone", "hey nimbus", "drone activate"],
    
    # Object Detection Configuration
    'GLOBAL_OBJECT': "car",
    'GLOBAL_INTENT': "",
    'GLOBAL_GET_DIST': 1,
    'GLOBAL_TARGET_DISTANCE': 0.0,
    'GLOBAL_STATE_ACTIVE': False,
    'GLOBAL_COMMAND_STATUS': "none",
    
    # Depth Detection Configuration
    'DEPTH_FRAME_COUNT': 5,  # Movement frames (1-5)
    'REFERENCE_FRAME_POSITION': 10,  # Which object detection gets reference frame
    
    # Processing Flags
    'OBJECT_DETECTION_BUSY': False,
    'DEPTH_PROCESSING_BUSY': False,
    
    # Video Source Configuration
    'USE_ROS2_VIDEO': False,  # Toggle: True = ROS2, False = Direct HTTP

    # ROS2 Configuration
    'ROS2_HOST': 'localhost',  # Docker exposes port 9090 to Windows
    'ROS2_PORT': 9090,
    'SIMULATION_MODE': True,  # Use simulation topics

    # Service Status
    'AI_SERVICE_RUNNING': False,
    'ROS2_SERVICE_RUNNING': False,
    'DISPLAY_SERVICE_RUNNING': False
})

# Thread lock for safe global state access
app.config['STATE_LOCK'] = threading.Lock()

# Import and register route blueprints
try:
    from routes.web_routes import web_bp
    from routes.ai_routes import ai_bp
    
    app.register_blueprint(web_bp)
    app.register_blueprint(ai_bp, url_prefix='/api')
    
    logger.info("Route blueprints registered successfully")
except ImportError as e:
    logger.error(f"Failed to import route blueprints: {e}")

# Import services
try:
    from services import ai_service, ros2_service, display_service, video_stream_service

    # Store service references in app context
    app.ai_service = ai_service
    app.ros2_service = ros2_service
    app.video_stream_service = video_stream_service
    app.display_service = display_service

    logger.info("Services imported successfully")
except ImportError as e:
    logger.error(f"Failed to import services: {e}")

def start_background_services():
    """Start all background services"""
    logger.info("Starting background services...")
    logger.info(f"Video source: {'ROS2' if app.config['USE_ROS2_VIDEO'] else 'Direct HTTP'}")

    try:
        # Start AI processing service
        if hasattr(app, 'ai_service'):
            app.ai_service.start_service(app)
            logger.info("AI service started")

        # Start video service (conditional based on USE_ROS2_VIDEO)
        if app.config['USE_ROS2_VIDEO']:
            if hasattr(app, 'ros2_service'):
                app.ros2_service.start_service(app)
                logger.info("ROS2 service started")
        else:
            if hasattr(app, 'video_stream_service'):
                app.video_stream_service.start_service(app, stream_url='http://localhost:8080/video')
                logger.info("Video stream service started (MJPEG from Webots)")

        # Start display service
        if hasattr(app, 'display_service'):
            app.display_service.start_service(app)
            logger.info("Display service started")

    except Exception as e:
        logger.error(f"Error starting background services: {e}")

def stop_background_services():
    """Stop all background services"""
    logger.info("Stopping background services...")
    
    try:
        if hasattr(app, 'ai_service'):
            app.ai_service.stop_service()
        
        if hasattr(app, 'ros2_service'):
            app.ros2_service.stop_service()
        
        if hasattr(app, 'display_service'):
            app.display_service.stop_service()
            
    except Exception as e:
        logger.error(f"Error stopping background services: {e}")

def initialize_application():
    """Initialize the application on startup"""
    logger.info("Initializing Nimbus AI application...")
    start_background_services()

@app.teardown_appcontext
def shutdown_application(exception):
    """Clean shutdown of services"""
    if exception:
        logger.error(f"Application shutdown due to exception: {exception}")

# Context processor to make config available in templates
@app.context_processor
def inject_config():
    return dict(config=app.config)

if __name__ == '__main__':
    logger.info("=" * 60)
    logger.info("NIMBUS AI SYSTEM - FLASK APPLICATION")
    logger.info("=" * 60)
    logger.info("Architecture: Flask-based web application with background services")
    logger.info("Web Interface: http://localhost:5000")
    logger.info("API Endpoints: http://localhost:5000/api/*")
    logger.info("")
    
    try:
        # Initialize services before starting Flask
        initialize_application()
        
        # Start the Flask application
        app.run(
            host='localhost',
            port=5000,
            debug=False,  # Set to False to avoid issues with threading
            threaded=True,
            use_reloader=False  # Disable reloader to prevent service duplication
        )
    except KeyboardInterrupt:
        logger.info("Application interrupted by user")
    except Exception as e:
        logger.error(f"Application error: {e}")
    finally:
        stop_background_services()
        logger.info("Nimbus AI System shutdown complete")