"""
Video Display Module
Handles displaying video frames with overlays and statistics
"""

import cv2
import time
import numpy as np
from typing import Optional, Dict
from datetime import datetime
from config import DISPLAY_SETTINGS

class VideoDisplay:
    """
    Video display class that handles showing frames with overlays
    """
    
    def __init__(self):
        self.window_name = DISPLAY_SETTINGS['window_name']
        self.window_created = False
        
        # FPS calculation
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        # Display state
        self.show_stats = True
        self.show_help = False
        
        # Colors (BGR format)
        self.colors = {
            'text': (255, 255, 255),      # White
            'background': (0, 0, 0),      # Black
            'success': (0, 255, 0),       # Green
            'warning': (0, 255, 255),     # Yellow
            'error': (0, 0, 255),         # Red
            'info': (255, 255, 0)         # Cyan
        }
        
        # Font settings
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.6
        self.font_thickness = 1
        self.font_scale_large = 0.8
        self.font_thickness_large = 2
    
    def create_window(self):
        """
        Create the display window
        """
        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            self.window_created = True
            
            # Set window size if specified
            if DISPLAY_SETTINGS['window_size']:
                width, height = DISPLAY_SETTINGS['window_size']
                cv2.resizeWindow(self.window_name, width, height)
    
    def show_frame(self, frame: np.ndarray, capture_stats: Dict) -> bool:
        """
        Display a frame with overlays and statistics
        
        Args:
            frame: The video frame to display
            capture_stats: Statistics from the capture module
            
        Returns:
            bool: True if should continue, False if user wants to quit
        """
        if frame is None:
            return True
        
        # Create window if needed
        self.create_window()
        
        # Create display frame with overlays
        display_frame = self._add_overlays(frame.copy(), capture_stats)
        
        # Show the frame
        cv2.imshow(self.window_name, display_frame)
        
        # Update FPS counter
        self._update_fps()
        
        # Handle key presses
        return self._handle_keys()
    
    def _add_overlays(self, frame: np.ndarray, stats: Dict) -> np.ndarray:
        """
        Add information overlays to the frame
        """
        height, width = frame.shape[:2]
        
        if self.show_stats:
            # Add main statistics
            self._add_main_stats(frame, stats)
            
            # Add connection info
            self._add_connection_info(frame, stats)
            
            # Add performance info
            self._add_performance_info(frame, stats)
        
        # Add timestamp
        if DISPLAY_SETTINGS['show_timestamp']:
            self._add_timestamp(frame)
        
        # Add help text
        if self.show_help:
            self._add_help_text(frame)
        else:
            # Just show basic controls
            self._add_basic_controls(frame)
        
        return frame
    
    def _add_main_stats(self, frame: np.ndarray, stats: Dict):
        """
        Add main statistics overlay
        """
        y_pos = 30
        
        # Connection status
        if stats['connected']:
            status_text = "CONNECTED"
            color = self.colors['success']
        else:
            status_text = "DISCONNECTED"
            color = self.colors['error']
        
        cv2.putText(frame, status_text, (10, y_pos), 
                   self.font, self.font_scale_large, color, self.font_thickness_large)
        y_pos += 35
        
        # Resolution and FPS
        if stats['connected']:
            res_text = f"Resolution: {stats['resolution']}"
            cv2.putText(frame, res_text, (10, y_pos), 
                       self.font, self.font_scale, self.colors['text'], self.font_thickness)
            y_pos += 25
            
            fps_text = f"Camera FPS: {stats['fps']} | Display FPS: {self.current_fps:.1f}"
            cv2.putText(frame, fps_text, (10, y_pos), 
                       self.font, self.font_scale, self.colors['text'], self.font_thickness)
            y_pos += 25
    
    def _add_connection_info(self, frame: np.ndarray, stats: Dict):
        """
        Add connection information
        """
        if not stats['connected']:
            return
        
        y_pos = 120
        
        # Backend info
        backend_text = f"Backend: {stats['backend']}"
        cv2.putText(frame, backend_text, (10, y_pos), 
                   self.font, self.font_scale, self.colors['info'], self.font_thickness)
        y_pos += 25
        
        # Uptime
        uptime = stats['uptime']
        uptime_text = f"Uptime: {uptime:.1f}s"
        cv2.putText(frame, uptime_text, (10, y_pos), 
                   self.font, self.font_scale, self.colors['info'], self.font_thickness)
    
    def _add_performance_info(self, frame: np.ndarray, stats: Dict):
        """
        Add performance information
        """
        if not stats['connected']:
            return
        
        height, width = frame.shape[:2]
        y_pos = height - 80
        
        # Frame counts
        frames_text = f"Frames: {stats['frames_captured']}"
        cv2.putText(frame, frames_text, (10, y_pos), 
                   self.font, self.font_scale, self.colors['text'], self.font_thickness)
        y_pos += 20
        
        # Dropped frames
        if stats['frames_dropped'] > 0:
            dropped_text = f"Dropped: {stats['frames_dropped']}"
            cv2.putText(frame, dropped_text, (10, y_pos), 
                       self.font, self.font_scale, self.colors['warning'], self.font_thickness)
        else:
            dropped_text = "Dropped: 0"
            cv2.putText(frame, dropped_text, (10, y_pos), 
                       self.font, self.font_scale, self.colors['success'], self.font_thickness)
    
    def _add_timestamp(self, frame: np.ndarray):
        """
        Add timestamp overlay
        """
        height, width = frame.shape[:2]
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Get text size to position it correctly
        (text_width, text_height), _ = cv2.getTextSize(
            timestamp, self.font, self.font_scale, self.font_thickness)
        
        x_pos = width - text_width - 10
        y_pos = 30
        
        cv2.putText(frame, timestamp, (x_pos, y_pos), 
                   self.font, self.font_scale, self.colors['text'], self.font_thickness)
    
    def _add_help_text(self, frame: np.ndarray):
        """
        Add help text overlay
        """
        help_lines = [
            "CAMERA TEST CONTROLS:",
            "",
            "Q - Quit",
            "H - Toggle this help",
            "S - Toggle statistics",
            "R - Reconnect camera",
            "SPACE - Take screenshot",
            "",
            "ESC - Quit"
        ]
        
        height, width = frame.shape[:2]
        
        # Calculate background size
        max_width = max(len(line) for line in help_lines) * 12
        bg_height = len(help_lines) * 25 + 20
        
        # Draw semi-transparent background
        overlay = frame.copy()
        cv2.rectangle(overlay, (width - max_width - 20, 10), 
                     (width - 10, bg_height + 10), self.colors['background'], -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Add help text
        y_pos = 35
        for line in help_lines:
            if line:  # Skip empty lines
                cv2.putText(frame, line, (width - max_width - 10, y_pos), 
                           self.font, self.font_scale, self.colors['text'], self.font_thickness)
            y_pos += 25
    
    def _add_basic_controls(self, frame: np.ndarray):
        """
        Add basic control information
        """
        height, width = frame.shape[:2]
        controls_text = "Controls: Q=quit, H=help, S=stats, R=reconnect"
        
        cv2.putText(frame, controls_text, (10, height - 20), 
                   self.font, self.font_scale, self.colors['text'], self.font_thickness)
    
    def _update_fps(self):
        """
        Update FPS calculation
        """
        self.fps_counter += 1
        
        if self.fps_counter >= DISPLAY_SETTINGS['fps_update_interval']:
            elapsed = time.time() - self.fps_start_time
            self.current_fps = self.fps_counter / elapsed
            
            # Reset counter
            self.fps_counter = 0
            self.fps_start_time = time.time()
    
    def _handle_keys(self) -> bool:
        """
        Handle keyboard input
        
        Returns:
            bool: True to continue, False to quit
        """
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:  # Q or Escape
            return False
        elif key == ord('h'):  # Toggle help
            self.show_help = not self.show_help
        elif key == ord('s'):  # Toggle stats
            self.show_stats = not self.show_stats
        elif key == ord('r'):  # Reconnect (handled by main program)
            return 'reconnect'
        elif key == ord(' '):  # Screenshot
            return 'screenshot'
        
        return True
    
    def show_message(self, message: str, duration: float = 2.0):
        """
        Show a temporary message
        """
        # Create a blank frame for the message
        msg_frame = np.zeros((200, 600, 3), dtype=np.uint8)
        
        # Add message text
        cv2.putText(msg_frame, message, (50, 100), 
                   self.font, self.font_scale_large, self.colors['text'], self.font_thickness_large)
        
        # Show message
        self.create_window()
        cv2.imshow(self.window_name, msg_frame)
        cv2.waitKey(int(duration * 1000))
    
    def cleanup(self):
        """
        Clean up display resources
        """
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            self.window_created = False