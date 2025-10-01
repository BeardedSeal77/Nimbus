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
        """Add SLAM processing overlay - DISABLED"""
        # SLAM overlay removed per user request
        pass
    
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
        
        # Simple clean overlay: Target and Distance only
        target_object = ai_result.get('target_object', '')
        target_distance = ai_result.get('target_distance')

        y_pos = 150

        # Target
        if target_object:
            cv2.putText(frame, f"Target: {target_object}", (10, y_pos),
                       self.font, 0.7, self.colors['info'], 2)
            y_pos += 30

        # Distance
        if target_distance is not None and target_distance > 0:
            cv2.putText(frame, f"Distance: {target_distance:.2f}m", (10, y_pos),
                       self.font, 0.7, self.colors['slam_good'], 2)
        else:
            cv2.putText(frame, f"Distance: --", (10, y_pos),
                       self.font, 0.7, (128, 128, 128), 2)
    
    
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