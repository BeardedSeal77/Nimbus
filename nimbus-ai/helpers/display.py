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
        status_color = self.colors['slam_good'] if 'object_detected' in processing_status else self.colors['slam_processing']
        
        if processing_status == 'object_detected':
            status_display = "Object Detected!"
        elif processing_status == 'object_detected_with_depth':
            status_display = "Object Detected + Distance Calculated!"
            status_color = self.colors['slam_good']
        elif processing_status == 'collecting_depth_frames':
            frames_collected = ai_result.get('depth_frames_collected') or 0
            frames_needed = ai_result.get('depth_frames_needed') or 5
            status_display = f"Collecting Depth Frames ({frames_collected}/{frames_needed})"
            status_color = self.colors['slam_processing']
        elif processing_status == 'object_detected_depth_busy':
            status_display = "Object Detected (Depth Processing Busy)"
            status_color = (0, 165, 255)  # Orange
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
        
        # Show depth collection status and distance information
        self._add_depth_info_overlay(frame, ai_result)
        
        # Show object pose if available
        object_pose = ai_result.get('object_pose')
        if object_pose:
            # Object has been located in 3D space
            pos = object_pose.get('position', {})
            distance = ((pos.get('x', 0)**2 + pos.get('y', 0)**2 + pos.get('z', 0)**2)**0.5)
            
            cv2.putText(frame, f"Target Distance: {distance:.2f}m", (10, 200), 
                       self.font, 0.6, self.colors['slam_good'], 1)
    
    def _add_depth_info_overlay(self, frame: np.ndarray, ai_result: Dict):
        """Add depth collection and distance information overlay"""
        y_pos = 200
        
        # Show current target distance if available
        target_distance = ai_result.get('target_distance')
        if target_distance is not None and target_distance > 0:
            cv2.putText(frame, f"Target Distance: {target_distance:.2f}m", 
                       (10, y_pos), self.font, 0.7, self.colors['slam_good'], 2)
            y_pos += 25
        
        # Show depth processing details if available
        depth_result = ai_result.get('depth_result')
        if depth_result:
            method = depth_result.get('method', 'unknown')
            confidence = depth_result.get('confidence', 0)
            frames_used = depth_result.get('frames_used', 0)
            
            # Method and confidence
            cv2.putText(frame, f"Method: {method} (Conf: {confidence:.2f})", 
                       (10, y_pos), self.font, 0.5, self.colors['info'], 1)
            y_pos += 20
            
            # Frames used
            cv2.putText(frame, f"Frames Used: {frames_used}", 
                       (10, y_pos), self.font, 0.5, self.colors['info'], 1)
            y_pos += 20
        
        # Show frame collection progress if collecting
        processing_status = ai_result.get('processing_status', '')
        if processing_status == 'collecting_depth_frames':
            frames_collected = ai_result.get('depth_frames_collected') or 0
            frames_needed = ai_result.get('depth_frames_needed') or 5
            progress = frames_collected / frames_needed if frames_needed > 0 else 0
            
            # Progress bar
            bar_width = 200
            bar_height = 10
            bar_x = 10
            bar_y = y_pos
            
            # Background
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), 
                         (50, 50, 50), -1)
            
            # Progress fill
            fill_width = int(bar_width * progress)
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_width, bar_y + bar_height), 
                         self.colors['slam_processing'], -1)
            
            # Progress text
            cv2.putText(frame, f"Depth Collection: {frames_collected}/{frames_needed}", 
                       (bar_x, bar_y - 5), self.font, 0.5, self.colors['info'], 1)
            y_pos += 30
        
        # Show depth estimation status and distance
        global_get_dist = ai_result.get('global_get_dist')
        target_distance = ai_result.get('target_distance')

        if global_get_dist is not None:
            if global_get_dist == 1:
                if target_distance is not None and target_distance > 0:
                    cv2.putText(frame, f"Distance to Target: {target_distance:.2f}m",
                               (10, y_pos), self.font, 0.5, self.colors['slam_good'], 1)
                else:
                    cv2.putText(frame, "Depth Detection: Active (collecting...)",
                               (10, y_pos), self.font, 0.5, self.colors['slam_good'], 1)
            else:
                cv2.putText(frame, "Depth Detection: DISABLED",
                           (10, y_pos), self.font, 0.5, (128, 128, 128), 1)
    
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