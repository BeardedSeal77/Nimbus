"""
AI Processing Service
Handles object detection and depth estimation in background thread
"""

import threading
import time
import logging
from queue import Queue, Empty
from datetime import datetime

logger = logging.getLogger(__name__)

class AIService:
    def __init__(self):
        self.app = None
        self.thread = None
        self.running = False
        
        # Frame processing
        self.frame_queue = Queue(maxsize=2)
        self.latest_detection_result = {}
        self.detection_result_lock = threading.Lock()
        
        # AI components
        self.object_detector = None
        self.depth_estimator = None
        
        # Counters
        self.detection_counter = 0
        self.successful_detection_counter = 0  # Only successful object detections
        
    def reset_depth_collection(self):
        """Reset depth collection when triggered from web interface"""
        self.successful_detection_counter = 0
        if self.depth_estimator:
            self.depth_estimator.reset_collection()
        logger.info("Depth collection reset for new cycle")
        
    def start_service(self, app):
        """Start the AI processing service"""
        self.app = app
        
        # Initialize AI components
        self._initialize_components()
        
        # Start processing thread
        self.running = True
        self.thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.thread.start()
        
        app.config['AI_SERVICE_RUNNING'] = True
        logger.info("AI processing service started")
        
    def stop_service(self):
        """Stop the AI processing service"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)
        
        if self.app:
            self.app.config['AI_SERVICE_RUNNING'] = False
        logger.info("AI processing service stopped")
        
    def _initialize_components(self):
        """Initialize object detector and depth estimator"""
        try:
            from classes.object_detect_node_v11 import ObjectDetector
            self.object_detector = ObjectDetector()
            logger.info("Object detector loaded")
        except ImportError as e:
            logger.warning(f"Object detector not available: {e}")
            
        try:
            from classes.depth_node import DepthEstimator
            self.depth_estimator = DepthEstimator()
            logger.info("Depth estimator loaded")
        except ImportError as e:
            logger.warning(f"Depth estimator not available: {e}")
    
    def add_frame_for_processing(self, frame):
        """Add frame to processing queue (called by ROS2 service)"""
        try:
            self.frame_queue.put_nowait(frame.copy())
        except:
            pass  # Queue full, skip frame
            
    def get_latest_detection_result(self):
        """Get latest detection result (thread-safe)"""
        with self.detection_result_lock:
            return self.latest_detection_result.copy()
    
    def _processing_loop(self):
        """Main AI processing loop"""
        logger.info("AI processing loop started with depth integration")
        last_processing_time = 0
        
        while self.running and self.app:
            try:
                current_time = time.time() * 1000
                interval = self.app.config['FRAME_PROCESSING_INTERVAL_MS']
                
                # Check if it's time for processing
                if (current_time - last_processing_time) >= interval:
                    try:
                        # Get latest frame from queue
                        frame = self.frame_queue.get_nowait()
                        
                        target_object = self.app.config['GLOBAL_OBJECT']
                        if target_object and self.object_detector:
                            self._process_frame_with_depth(frame, target_object)
                            last_processing_time = current_time
                        
                    except Empty:
                        pass  # No frame available
                    except Exception as e:
                        logger.error(f"Frame processing error: {e}")
                        self._update_detection_result({
                            'bounding_box': None,
                            'processing_status': 'detection_error'
                        })
                
                time.sleep(0.01)  # Small sleep to prevent busy waiting
                
            except Exception as e:
                logger.error(f"AI processing loop error: {e}")
                time.sleep(0.1)
        
        logger.info("AI processing loop stopped")
    
    def _process_frame_with_depth(self, frame, target_object):
        """Process frame with object detection and optional depth estimation"""
        # Step 1: Object Detection (with safety flag)
        if not self.app.config['OBJECT_DETECTION_BUSY']:
            self.app.config['OBJECT_DETECTION_BUSY'] = True
            try:
                detection_result = self.object_detector.detect(target_object, frame)
            finally:
                self.app.config['OBJECT_DETECTION_BUSY'] = False
            
            if detection_result:  # Object found
                self.detection_counter += 1
                self.successful_detection_counter += 1  # Count successful detections
                
                # Create detection result data
                detection_data = {
                    'x': detection_result['x'],
                    'y': detection_result['y'], 
                    'width': detection_result['width'],
                    'height': detection_result['height'],
                    'object_name': target_object,
                    'confidence': 0.85
                }
                
                # Step 2: Depth Collection (if enabled) - branches off object detection
                self._handle_depth_processing(frame, detection_result, detection_data)
                
                logger.debug(f"Object detected: {target_object} (detection #{self.successful_detection_counter})")
            else:
                # No object found - increment total counter but not successful counter
                self.detection_counter += 1
                self._update_detection_result({
                    'bounding_box': None,
                    'processing_status': 'object_searching'
                })
        else:
            logger.debug("Object detection busy, skipping frame")
    
    def _handle_depth_processing(self, frame, detection_result, detection_data):
        """Handle depth processing logic - branches off successful object detection"""
        get_dist = self.app.config['GLOBAL_GET_DIST']
        depth_busy = self.app.config['DEPTH_PROCESSING_BUSY']
        
        if get_dist == 1 and self.depth_estimator and not depth_busy:
            # Check if we should collect this successful detection for depth
            should_collect = self.depth_estimator.should_collect_frame(self.successful_detection_counter)
            
            if should_collect:
                logger.debug(f"Collecting detection #{self.successful_detection_counter} for depth processing")
                depth_result = self.depth_estimator.collect_frame(frame, detection_result, self.successful_detection_counter)
                
                if depth_result:  # All frames collected, processing complete
                    with self.app.config['STATE_LOCK']:
                        self.app.config['GLOBAL_TARGET_DISTANCE'] = depth_result['distance']
                        self.app.config['GLOBAL_GET_DIST'] = 0  # Auto-disable
                    
                    # Reset successful detection counter for next depth cycle
                    self.successful_detection_counter = 0
                    
                    logger.info(f"Depth processing complete: {depth_result['distance']:.2f}m")
                    
                    self._update_detection_result({
                        'bounding_box': detection_data,
                        'processing_status': 'object_detected_with_depth',
                        'depth_result': depth_result,
                        'target_distance': depth_result['distance']
                    })
                else:
                    # Still collecting frames - show collection status
                    collection_status = self.depth_estimator.get_collection_status()
                    
                    if collection_status['waiting_for_reference']:
                        status_text = 'waiting_for_reference_frame'
                        logger.debug(f"Waiting for reference frame (detection #{self.app.config['REFERENCE_FRAME_POSITION']})")
                    else:
                        status_text = 'collecting_depth_frames'
                    
                    self._update_detection_result({
                        'bounding_box': detection_data,
                        'processing_status': status_text,
                        'depth_collection_status': collection_status,
                        'current_detection_number': self.successful_detection_counter
                    })
            else:
                # Object detected but not collecting this detection (frames 6-9)
                collection_status = self.depth_estimator.get_collection_status()
                
                if collection_status['waiting_for_reference']:
                    status_text = 'waiting_for_reference_frame'
                else:
                    status_text = 'object_detected'
                
                self._update_detection_result({
                    'bounding_box': detection_data,
                    'processing_status': status_text,
                    'depth_collection_status': collection_status,
                    'current_detection_number': self.successful_detection_counter
                })
        elif get_dist == 1 and depth_busy:
            # Skip depth if busy - object detection continues normally
            self._update_detection_result({
                'bounding_box': detection_data,
                'processing_status': 'object_detected_depth_busy'
            })
        else:
            # Depth collection disabled - show previous distance if available
            target_distance = self.app.config['GLOBAL_TARGET_DISTANCE']
            self._update_detection_result({
                'bounding_box': detection_data,
                'processing_status': 'object_detected',
                'target_distance': target_distance if target_distance > 0 else None
            })
    
    def _update_detection_result(self, result):
        """Update detection result (thread-safe)"""
        with self.detection_result_lock:
            self.latest_detection_result = result

# Global service instance
ai_service_instance = AIService()

def start_service(app):
    """Start the AI service"""
    ai_service_instance.start_service(app)

def stop_service():
    """Stop the AI service"""
    ai_service_instance.stop_service()

def add_frame_for_processing(frame):
    """Add frame for processing"""
    ai_service_instance.add_frame_for_processing(frame)

def get_latest_detection_result():
    """Get latest detection result"""
    return ai_service_instance.get_latest_detection_result()

def reset_depth_collection():
    """Reset depth collection for new cycle"""
    ai_service_instance.reset_depth_collection()