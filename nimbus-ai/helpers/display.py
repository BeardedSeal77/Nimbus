"""
Processed Display Helper
Creates visual overlays for AI processing results
Shows bounding boxes, SLAM pose, object detection, etc.
"""

import cv2
import numpy as np
from typing import Dict, Optional, Any
from datetime import datetime

class ProcessedDisplay:
    """
    Display helper for AI processing results
    Creates visual overlays on camera feed
    """
    
    def __init__(self):
        # Colors for different elements (BGR format)
        self.colors = {
            'slam_good': (0, 255, 0),      # Green
            'slam_bad': (0, 0, 255),       # Red  
            'slam_processing': (0, 255, 255), # Yellow
            'bbox': (255, 0, 0),           # Blue
            'text': (255, 255, 255),       # White
            'background': (0, 0, 0),       # Black
            'info': (255, 255, 0)          # Cyan
        }
        
        # Fonts
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.7
        self.font_thickness = 2
        
    def create_display_frame(self, frame: np.ndarray, ai_result: Dict, 
                           frame_count: int, elapsed_time: float) -> np.ndarray:
        """
        Create display frame with AI processing overlays
        
        Args:
            frame: Original camera frame
            ai_result: AI processing results
            frame_count: Current frame number
            elapsed_time: Time since start
            
        Returns:
            np.ndarray: Frame with overlays
        """
        display_frame = frame.copy()
        
        # Add frame info
        self._add_frame_info(display_frame, frame_count, elapsed_time)
        
        # Add SLAM info
        self._add_slam_overlay(display_frame, ai_result)
        
        # Add object detection overlays
        self._add_object_detection_overlay(display_frame, ai_result)
        
        # Add system status
        self._add_system_status(display_frame, ai_result)
        
        return display_frame
    
    def _add_frame_info(self, frame: np.ndarray, frame_count: int, elapsed_time: float):
        """Add basic frame information"""
        height, width = frame.shape[:2]
        fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        
        # Frame counter and FPS
        text = f"Frame: {frame_count} | FPS: {fps:.1f}"
        cv2.putText(frame, text, (10, 30), self.font, self.font_scale, 
                   self.colors['info'], self.font_thickness)
        
        # Timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        cv2.putText(frame, timestamp, (width - 120, 30), self.font, 0.6, 
                   self.colors['info'], 1)
    
    def _add_slam_overlay(self, frame: np.ndarray, ai_result: Dict):
        """Add SLAM processing overlay"""
        camera_pose = ai_result.get('camera_pose')
        
        if camera_pose:
            # SLAM is working
            cv2.putText(frame, "SLAM: OK", (10, 70), self.font, self.font_scale,
                       self.colors['slam_good'], self.font_thickness)
            
            # Show position if available
            if 'position' in camera_pose:
                pos = camera_pose['position']
                pos_text = f"Pos: ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})"
                cv2.putText(frame, pos_text, (10, 100), self.font, 0.6,
                           self.colors['slam_good'], 1)
            
            # Show confidence if available
            if 'confidence' in camera_pose:
                conf = camera_pose['confidence']
                conf_text = f"Conf: {conf:.2f}"
                cv2.putText(frame, conf_text, (10, 125), self.font, 0.6,
                           self.colors['slam_good'], 1)
        else:
            # SLAM not working or processing
            cv2.putText(frame, "SLAM: Processing...", (10, 70), self.font, self.font_scale,
                       self.colors['slam_processing'], self.font_thickness)
    
    def _add_object_detection_overlay(self, frame: np.ndarray, ai_result: Dict):
        """Add object detection bounding boxes and labels"""
        bounding_box = ai_result.get('bounding_box')
        
        if bounding_box:
            # Draw bounding box
            x = int(bounding_box.get('x', 0))
            y = int(bounding_box.get('y', 0))
            width = int(bounding_box.get('width', 0))
            height = int(bounding_box.get('height', 0))
            
            # Draw rectangle
            cv2.rectangle(frame, (x, y), (x + width, y + height), 
                         self.colors['bbox'], 2)
            
            # Add label if available
            if 'object_name' in bounding_box:
                label = bounding_box['object_name']
                confidence = bounding_box.get('confidence', 0)
                label_text = f"{label} ({confidence:.2f})"
                
                # Add background for text
                (text_width, text_height), _ = cv2.getTextSize(
                    label_text, self.font, 0.6, 1)
                cv2.rectangle(frame, (x, y - text_height - 10), 
                             (x + text_width, y), self.colors['bbox'], -1)
                
                # Add text
                cv2.putText(frame, label_text, (x, y - 5), self.font, 0.6,
                           self.colors['text'], 1)
        
        # Show target object status and search info
        processing_status = ai_result.get('processing_status', 'unknown')
        
        # Show current global state (passed via ai_result to avoid circular import)
        target_object = ai_result.get('target_object', '')
        intent = ai_result.get('intent', '')
        if target_object:
            status_text = f"Target: {target_object} | Intent: {intent}"
            cv2.putText(frame, status_text, (10, 150), self.font, 0.6, 
                       self.colors['info'], 1)
        
        # Show processing status
        status_color = self.colors['slam_good'] if processing_status == 'object_detected' else self.colors['slam_processing']
        if processing_status == 'object_detected':
            status_display = "Object Detected!"
        elif processing_status == 'object_searching':
            status_display = "Searching for object..."
        elif processing_status == 'ai_disabled':
            status_display = "AI Processing Disabled"
        elif processing_status == 'detection_error':
            status_display = "Detection Error"
        else:
            status_display = "Live Stream"
            
        cv2.putText(frame, f"Status: {status_display}", (10, 175), 
                   self.font, 0.6, status_color, 1)
        
        # Show object pose if available
        object_pose = ai_result.get('object_pose')
        if object_pose:
            # Object has been located in 3D space
            pos = object_pose.get('position', {})
            distance = ((pos.get('x', 0)**2 + pos.get('y', 0)**2 + pos.get('z', 0)**2)**0.5)
            
            cv2.putText(frame, f"Target Distance: {distance:.2f}m", (10, 200), 
                       self.font, 0.6, self.colors['slam_good'], 1)
    
    def _add_system_status(self, frame: np.ndarray, ai_result: Dict):
        """Add overall system status"""
        height, width = frame.shape[:2]
        
        # Add instructions at bottom
        instructions = [
            "Controls: 'q' = quit, 's' = toggle SLAM, 'r' = reset",
            "Nimbus AI Live System - Camera + AI Processing"
        ]
        
        y_pos = height - 50
        for instruction in instructions:
            cv2.putText(frame, instruction, (10, y_pos), self.font, 0.5,
                       self.colors['text'], 1)
            y_pos += 20
    
    def draw_trajectory(self, frame: np.ndarray, poses: list, color=None):
        """
        Draw SLAM trajectory on frame
        
        Args:
            frame: Frame to draw on
            poses: List of camera poses
            color: Line color (optional)
        """
        if len(poses) < 2:
            return
        
        if color is None:
            color = self.colors['slam_good']
        
        # Convert 3D poses to 2D screen coordinates (simplified projection)
        points = []
        for pose in poses:
            if 'position' in pose:
                pos = pose['position']
                # Simple orthographic projection (you might want to improve this)
                screen_x = int(pos['x'] * 100 + frame.shape[1] // 2)
                screen_y = int(pos['z'] * 100 + frame.shape[0] // 2)
                points.append((screen_x, screen_y))
        
        # Draw trajectory lines
        for i in range(1, len(points)):
            cv2.line(frame, points[i-1], points[i], color, 2)
    
    def add_custom_overlay(self, frame: np.ndarray, text: str, position: tuple, 
                          color=None, size=None):
        """
        Add custom text overlay
        
        Args:
            frame: Frame to draw on
            text: Text to display
            position: (x, y) position
            color: Text color (optional)
            size: Font size (optional)
        """
        if color is None:
            color = self.colors['text']
        
        if size is None:
            size = self.font_scale
        
        cv2.putText(frame, text, position, self.font, size, color, self.font_thickness)