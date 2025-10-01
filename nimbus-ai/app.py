#!/usr/bin/env python3
"""
Nimbus AI Worker Process
Runs as child process spawned by nimbus-hub
Performs AI/ML processing using shared state for communication
"""

import os
import sys
import time
import logging
import traceback
import threading

# Add project paths
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.append(project_root)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - [AI WORKER] - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def run_ai_worker(shared_state):
    """
    Main AI worker function
    Runs in separate process, communicates via shared_state

    Args:
        shared_state: multiprocessing.Manager().dict() for IPC
    """
    logger.info("=" * 60)
    logger.info("NIMBUS AI WORKER PROCESS STARTING")
    logger.info("=" * 60)

    try:
        # Import services
        from services import ai_service, ros2_service, display_service, video_stream_service
        from services.audio_service import AudioService

        logger.info("Services imported successfully")

        # Create pseudo-app config for services
        class WorkerConfig:
            """Config object that mimics Flask app.config"""
            def __init__(self, shared_state):
                self.shared_state = shared_state
                self.config = {
                    # AI Processing Configuration
                    'FRAME_PROCESSING_INTERVAL_MS': 100,
                    'ACTIVATION_PHRASES': ["ok drone", "hey nimbus", "drone activate"],

                    # Object Detection Configuration (read from shared state)
                    'GLOBAL_OBJECT': shared_state.get('target_object', 'car'),
                    'GLOBAL_INTENT': shared_state.get('global_intent', ''),
                    'GLOBAL_GET_DIST': shared_state.get('global_get_dist', 1),
                    'GLOBAL_TARGET_DISTANCE': shared_state.get('global_target_distance', 0.0),
                    'GLOBAL_STATE_ACTIVE': shared_state.get('global_state_active', False),
                    'GLOBAL_COMMAND_STATUS': shared_state.get('global_command_status', 'none'),

                    # Depth Detection Configuration
                    'DEPTH_FRAME_COUNT': 5,
                    'REFERENCE_FRAME_POSITION': 10,

                    # Processing Flags
                    'OBJECT_DETECTION_BUSY': False,
                    'DEPTH_PROCESSING_BUSY': False,

                    # Thread Safety
                    'STATE_LOCK': threading.Lock(),

                    # Video Source Configuration
                    'USE_ROS2_VIDEO': False,  # Use direct HTTP

                    # ROS2 Configuration
                    'ROS2_HOST': 'localhost',
                    'ROS2_PORT': 9090,
                    'SIMULATION_MODE': True,

                    # Service Status
                    'AI_SERVICE_RUNNING': False,
                    'ROS2_SERVICE_RUNNING': False,
                    'DISPLAY_SERVICE_RUNNING': False
                }

            def get(self, key, default=None):
                return self.config.get(key, default)

            def __getitem__(self, key):
                return self.config[key]

            def __setitem__(self, key, value):
                self.config[key] = value

            def update(self, *args, **kwargs):
                self.config.update(*args, **kwargs)

        # Create worker config
        worker_config = WorkerConfig(shared_state)

        logger.info("Starting background services...")

        # Start video stream service
        if hasattr(video_stream_service, 'start_service'):
            video_stream_service.start_service(worker_config, stream_url='http://localhost:8080/video')
            logger.info("Video stream service started (MJPEG from Webots)")

        # Start AI service
        if hasattr(ai_service, 'start_service'):
            ai_service.start_service(worker_config)
            logger.info("AI service started")

        # Start display service
        if hasattr(display_service, 'start_service'):
            display_service.start_service(worker_config)
            logger.info("Display service started")

        # Initialize audio service
        audio_service_instance = AudioService()
        audio_service_instance.initialize(worker_config)
        logger.info("Audio service initialized")

        logger.info("All services started successfully")
        logger.info("Entering main processing loop...")

        # Main processing loop
        frame_count = 0
        last_fps_time = time.time()

        while shared_state.get('ai_running', True):
            try:
                # Update heartbeat
                shared_state['last_heartbeat'] = time.time()

                # Check for cancel signal
                if shared_state.get('cancel', False):
                    logger.info("Cancel signal received, pausing processing")
                    shared_state['cancel'] = False
                    time.sleep(0.5)
                    continue

                # Sync configuration from shared state
                # Read from both 'global_object' (audio) and 'target_object' (web UI)
                worker_config['GLOBAL_OBJECT'] = shared_state.get('global_object', shared_state.get('target_object', 'car'))
                worker_config['GLOBAL_INTENT'] = shared_state.get('global_intent', '')
                worker_config['GLOBAL_GET_DIST'] = shared_state.get('global_get_dist', 0)
                worker_config['mode'] = shared_state.get('mode', 'MANUAL')

                # Handle audio recording control
                if shared_state.get('audio_recording', False) and not audio_service_instance.recording:
                    audio_service_instance.start_recording()
                elif not shared_state.get('audio_recording', False) and audio_service_instance.recording:
                    pass

                # Handle audio processing trigger
                if shared_state.get('audio_process_trigger', False):
                    shared_state['audio_process_trigger'] = False
                    result = audio_service_instance.stop_recording()
                    shared_state['audio_result'] = result

                # Calculate FPS
                frame_count += 1
                if frame_count % 30 == 0:
                    current_time = time.time()
                    fps = 30.0 / (current_time - last_fps_time)
                    shared_state['processing_fps'] = fps
                    last_fps_time = current_time

                # Get latest AI results if available
                try:
                    if hasattr(ai_service, 'get_latest_detection_result'):
                        result = ai_service.get_latest_detection_result()
                        if result:
                            shared_state['ai_results'] = result
                            if 'detections' in result:
                                shared_state['detection_count'] = len(result['detections'])
                except Exception as e:
                    logger.debug(f"Error getting AI results: {e}")

                # Get latest processed frame for video streaming
                try:
                    # Get PROCESSED display frame (with overlays, bounding boxes, etc.)
                    if hasattr(display_service, 'display_service_instance'):
                        display_inst = display_service.display_service_instance
                        with display_inst.frame_lock:
                            if display_inst.current_display_frame is not None:
                                # Encode processed frame as JPEG for streaming
                                import cv2
                                ret, jpeg = cv2.imencode('.jpg', display_inst.current_display_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                                if ret:
                                    shared_state['video_frame'] = jpeg.tobytes()
                except Exception as e:
                    logger.debug(f"Error encoding video frame: {e}")

                # Small sleep to prevent CPU hammering
                time.sleep(0.01)  # 100Hz loop

            except KeyboardInterrupt:
                logger.info("Worker interrupted by keyboard")
                break
            except Exception as e:
                logger.error(f"Error in worker loop: {e}")
                logger.error(traceback.format_exc())
                time.sleep(1)  # Prevent runaway on persistent errors

        logger.info("AI worker shutting down...")

        # Stop services
        try:
            if hasattr(ai_service, 'stop_service'):
                ai_service.stop_service()
            if hasattr(display_service, 'stop_service'):
                display_service.stop_service()
            if hasattr(video_stream_service, 'stop_service'):
                video_stream_service.stop_service()
        except Exception as e:
            logger.error(f"Error stopping services: {e}")

        logger.info("AI worker process stopped")

    except Exception as e:
        logger.error(f"Fatal error in AI worker: {e}")
        logger.error(traceback.format_exc())
        shared_state['ai_running'] = False

if __name__ == '__main__':
    logger.error("This module should not be run directly")
    logger.error("It is spawned as a child process by nimbus-hub/hub.py")
    sys.exit(1)