# nimbus/AI/AI.py
# Nimbus AI Main Aggregator
# Central coordinator for all AI processes in the drone control system.
# Manages state, data flow, and orchestrates all AI nodes.

import json
import logging
import threading
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, asdict
from datetime import datetime
import numpy as np

# Import AI processing nodes
from classes.stt_node import stt_node
from classes.intent_object_node import intent_object_node
from classes.object_detect_node import object_detect_node
from classes.depth_node import depth_node
from classes.rtabmap_node import rtabmap_node
from classes.survey_node import survey_node

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class DroneState:
    # Data class to hold all drone state information
    # Audio/Text Processing
    transcript: str = ""
    intent: str = ""
    target_object: str = ""
    
    # Visual Processing
    current_frame: Optional[np.ndarray] = None
    bounding_box: Optional[Dict[str, float]] = None
    detected_objects: List[Dict] = None
    
    # Spatial Information
    camera_pose: Optional[Dict[str, float]] = None  # Current camera position/orientation
    drone_position: Optional[Dict[str, float]] = None  # Current drone position
    object_pose: Optional[Dict[str, float]] = None  # Target object 3D position
    origin_position: Dict[str, float] = None  # Where drone started (home)
    
    # Environmental Data
    depth_data: Optional[np.ndarray] = None
    scene_objects: List[Dict] = None  # All objects in environment from survey
    
    # System Status
    last_updated: datetime = None
    processing_status: str = "idle"  # idle, processing, error, complete
    error_message: str = ""
    
    def __post_init__(self):
        if self.detected_objects is None:
            self.detected_objects = []
        if self.scene_objects is None:
            self.scene_objects = []
        if self.last_updated is None:
            self.last_updated = datetime.now()

class NimbusAI:
    # Main AI aggregator class for the Nimbus drone system.
    # Coordinates all AI processing nodes and maintains system state.
    
    def __init__(self):
        self.state = DroneState()
        self.is_running = False
        self._lock = threading.Lock()
        
        # Processing flags
        self.continuous_slam = False
        self.auto_survey = False
        
        # Performance monitoring
        self.processing_times = {}
        self.node_status = {
            'stt': 'ready',
            'intent_object': 'ready',
            'object_detect': 'ready',
            'depth': 'ready',
            'rtabmap': 'ready',
            'survey': 'ready'
        }
        
        logger.info("Nimbus AI system initialized")
    
    def set_origin(self, position: Dict[str, float]):
        # Set the drone's origin/home position
        with self._lock:
            self.state.origin_position = position.copy()
            logger.info(f"Origin set to: {position}")
    
    def get_state(self) -> Dict:
        # Get current system state as dictionary
        with self._lock:
            return asdict(self.state)
    
    def update_status(self, status: str, error_msg: str = ""):
        # Update processing status
        with self._lock:
            self.state.processing_status = status
            self.state.error_message = error_msg
            self.state.last_updated = datetime.now()
    
    # Audio Processing Pipeline
    def process_audio(self, audio_data: np.ndarray) -> Dict[str, str]:
        # Process audio input through STT and intent/object extraction
        # Returns: {'intent': str, 'object': str, 'transcript': str}
        try:
            self.update_status("processing")
            start_time = time.time()
            
            # Step 1: Speech to Text
            logger.info("Processing audio through STT...")
            self.node_status['stt'] = 'processing'
            transcript = stt_node(audio_data)
            self.node_status['stt'] = 'complete'
            
            with self._lock:
                self.state.transcript = transcript
            
            logger.info(f"Transcript: {transcript}")
            
            # Step 2: Extract Intent and Object
            logger.info("Extracting intent and object...")
            self.node_status['intent_object'] = 'processing'
            intent_object_result = intent_object_node(transcript)
            self.node_status['intent_object'] = 'complete'
            
            with self._lock:
                self.state.intent = intent_object_result.get('intent', '')
                self.state.target_object = intent_object_result.get('object', '')
            
            # Record processing time
            self.processing_times['audio_pipeline'] = time.time() - start_time
            
            result = {
                'intent': self.state.intent,
                'object': self.state.target_object,
                'transcript': self.state.transcript
            }
            
            logger.info(f"Audio processing complete: {result}")
            self.update_status("complete")
            
            return result
            
        except Exception as e:
            error_msg = f"Audio processing failed: {str(e)}"
            logger.error(error_msg)
            self.update_status("error", error_msg)
            self.node_status['stt'] = 'error'
            self.node_status['intent_object'] = 'error'
            raise
    
    # Visual Processing Pipeline
    def process_frame(self, frame: np.ndarray, depth_data: Optional[np.ndarray] = None) -> Dict:
        # Process camera frame for object detection and pose estimation
        # Returns: {'bounding_box': dict, 'object_pose': dict, 'camera_pose': dict}
        try:
            self.update_status("processing")
            start_time = time.time()
            
            with self._lock:
                self.state.current_frame = frame.copy()
                if depth_data is not None:
                    self.state.depth_data = depth_data.copy()
            
            # Step 1: SLAM for camera pose (runs continuously)
            if self.continuous_slam:
                logger.debug("Updating camera pose via SLAM...")
                self.node_status['rtabmap'] = 'processing'
                camera_pose = rtabmap_node(frame, depth_data)
                self.node_status['rtabmap'] = 'complete'
                
                with self._lock:
                    self.state.camera_pose = camera_pose
                    self.state.drone_position = camera_pose  # Assuming camera is on drone
            
            # Step 2: Object Detection (if we have a target)
            bounding_box = None
            object_pose = None
            
            if self.state.intent and self.state.target_object:
                logger.info(f"Detecting object: {self.state.target_object}")
                self.node_status['object_detect'] = 'processing'
                bounding_box = object_detect_node(
                    self.state.intent, 
                    self.state.target_object, 
                    frame
                )
                self.node_status['object_detect'] = 'complete'
                
                with self._lock:
                    self.state.bounding_box = bounding_box
                
                # Step 3: Depth estimation for 3D position
                if bounding_box and self.state.camera_pose:
                    logger.info("Calculating object 3D position...")
                    self.node_status['depth'] = 'processing'
                    object_pose = depth_node(
                        self.state.intent,
                        bounding_box,
                        frame,
                        self.state.camera_pose
                    )
                    self.node_status['depth'] = 'complete'
                    
                    with self._lock:
                        self.state.object_pose = object_pose
            
            # Record processing time
            self.processing_times['visual_pipeline'] = time.time() - start_time
            
            result = {
                'bounding_box': bounding_box,
                'object_pose': object_pose,
                'camera_pose': self.state.camera_pose
            }
            
            logger.debug(f"Frame processing complete")
            self.update_status("complete")
            
            return result
            
        except Exception as e:
            error_msg = f"Frame processing failed: {str(e)}"
            logger.error(error_msg)
            self.update_status("error", error_msg)
            for node in ['rtabmap', 'object_detect', 'depth']:
                if self.node_status[node] == 'processing':
                    self.node_status[node] = 'error'
            raise
    
    # Survey Operations
    def run_survey(self, frame: np.ndarray, depth_data: np.ndarray) -> List[Dict]:
        # Run comprehensive environment survey
        # Returns: List of all detected objects with poses
        try:
            self.update_status("processing")
            start_time = time.time()
            
            logger.info("Starting environment survey...")
            self.node_status['survey'] = 'processing'
            
            scene_objects = survey_node(frame, depth_data, self.state.camera_pose)
            
            with self._lock:
                self.state.scene_objects = scene_objects
            
            self.node_status['survey'] = 'complete'
            self.processing_times['survey'] = time.time() - start_time
            
            logger.info(f"Survey complete: found {len(scene_objects)} objects")
            self.update_status("complete")
            
            return scene_objects
            
        except Exception as e:
            error_msg = f"Survey failed: {str(e)}"
            logger.error(error_msg)
            self.update_status("error", error_msg)
            self.node_status['survey'] = 'error'
            raise
    
    # Main Processing Methods
    def process_voice_command(self, audio_data: np.ndarray) -> Dict:
        # Complete voice command processing pipeline
        # Returns: Command processing result with navigation data
        logger.info("Processing voice command...")
        
        # Process audio to get intent and object
        audio_result = self.process_audio(audio_data)
        
        # Handle different intents
        intent = audio_result['intent'].lower()
        
        if intent == 'home':
            return self._handle_home_command()
        elif intent == 'cancel':
            return self._handle_cancel_command()
        elif intent == 'go':
            return self._handle_go_command(audio_result['object'])
        else:
            return {'status': 'error', 'message': f'Unknown intent: {intent}'}
    
    def _handle_home_command(self) -> Dict:
        # Handle 'home' intent
        if not self.state.origin_position:
            return {'status': 'error', 'message': 'Origin position not set'}
        
        return {
            'status': 'success',
            'action': 'navigate_to',
            'target_pose': self.state.origin_position,
            'message': 'Returning to origin'
        }
    
    def _handle_cancel_command(self) -> Dict:
        # Handle 'cancel' intent
        with self._lock:
            self.state.intent = ""
            self.state.target_object = ""
            self.state.bounding_box = None
            self.state.object_pose = None
        
        return {
            'status': 'success',
            'action': 'cancel',
            'message': 'Command cancelled'
        }
    
    def _handle_go_command(self, target_object: str) -> Dict:
        # Handle 'go' intent with target object
        if not target_object:
            return {'status': 'error', 'message': 'No target object specified'}
        
        # Object detection and pose estimation will happen in process_frame()
        # This just sets up the target
        return {
            'status': 'success',
            'action': 'search_and_navigate',
            'target_object': target_object,
            'message': f'Searching for {target_object}'
        }
    
    # Continuous Processing
    def start_continuous_slam(self):
        # Enable continuous SLAM processing
        self.continuous_slam = True
        logger.info("Continuous SLAM enabled")
    
    def stop_continuous_slam(self):
        # Disable continuous SLAM processing
        self.continuous_slam = False
        logger.info("Continuous SLAM disabled")
    
    def enable_auto_survey(self):
        # Enable automatic environment surveying
        self.auto_survey = True
        logger.info("Auto survey enabled")
    
    def disable_auto_survey(self):
        # Disable automatic environment surveying
        self.auto_survey = False
        logger.info("Auto survey disabled")
    
    # Utility Methods
    def get_performance_metrics(self) -> Dict:
        # Get system performance metrics
        return {
            'processing_times': self.processing_times,
            'node_status': self.node_status,
            'system_status': self.state.processing_status,
            'last_updated': self.state.last_updated.isoformat() if self.state.last_updated else None
        }
    
    def reset_state(self):
        # Reset all state data except origin
        origin = self.state.origin_position
        with self._lock:
            self.state = DroneState()
            self.state.origin_position = origin
        logger.info("AI state reset")
    
    def get_navigation_data(self) -> Optional[Dict]:
        # Get current navigation target data
        if not self.state.object_pose:
            return None
        
        return {
            'target_pose': self.state.object_pose,
            'current_pose': self.state.drone_position,
            'origin_pose': self.state.origin_position,
            'target_object': self.state.target_object,
            'intent': self.state.intent
        }
    
    def health_check(self) -> Dict:
        # System health check
        healthy_nodes = sum(1 for status in self.node_status.values() 
                          if status in ['ready', 'complete'])
        total_nodes = len(self.node_status)
        
        return {
            'status': 'healthy' if self.state.processing_status != 'error' else 'error',
            'nodes_healthy': f"{healthy_nodes}/{total_nodes}",
            'node_status': self.node_status,
            'error_message': self.state.error_message,
            'uptime': time.time()  # Could track actual uptime
        }

# Singleton instance for global access
ai_system = NimbusAI()

def get_ai_system() -> NimbusAI:
    # Get the global AI system instance
    return ai_system