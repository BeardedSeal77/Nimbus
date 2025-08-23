# use: OpenCV, RTAB-Map
# inputs: intent, bounding_box, video, camera ProcessLookupError
# output: detected object (object pose)

import numpy as np
import cv2
import logging
from datetime import datetime
from typing import Dict, List, Optional, Any

# Configure logging
logger = logging.getLogger(__name__)

# Global variables from integration plan
DEPTH_FRAME_COUNT = 5     # Collect 5 movement frames for SfM
REFERENCE_FRAME_POSITION = 10  # Which detection gets reference frame
DEPTH_PROCESSING_BUSY = False

# Fallback camera intrinsics (640x480 @ 60° FOV typical RGB-D camera)
CAMERA_INTRINSICS = {
    'fx': 525.0,  # Focal length x
    'fy': 525.0,  # Focal length y
    'cx': 320.0,  # Principal point x (image center)
    'cy': 240.0   # Principal point y (image center)
}

def depth_node(intent: str,
               bounding_box: dict,
               rgb_frame: np.ndarray,
               camera_pose: dict,
               depth_map: np.ndarray = None) -> dict:
    """
    Estimate the 3D world position of the target object using depth data.

    Parameters:
        intent (str): User intent (not used yet).
        bounding_box (dict): Detected 2D bounding box {'x', 'y', 'width', 'height'}.
        rgb_frame (np.ndarray): BGR image (unused but available).
        camera_pose (dict): Estimated camera pose in world coordinates.
        depth_map (np.ndarray): Depth image (float32, meters).

    Returns:
        dict: {'x', 'y', 'z'} in world coordinates or empty dict if failed.
    """
    try:
        if depth_map is None:
            raise ValueError("Depth map is required")

        # Extract center of bounding box
        x1 = int(bounding_box['x'])
        y1 = int(bounding_box['y'])
        x2 = x1 + int(bounding_box['width'])
        y2 = y1 + int(bounding_box['height'])
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Get depth at center
        depth = float(depth_map[cy, cx])
        if np.isnan(depth) or depth <= 0.0:
            raise ValueError("Invalid depth at object center")

        # Camera intrinsics
        fx = CAMERA_INTRINSICS['fx']
        fy = CAMERA_INTRINSICS['fy']
        cx_i = CAMERA_INTRINSICS['cx']
        cy_i = CAMERA_INTRINSICS['cy']

        # Convert to camera space (3D)
        x_cam = (cx - cx_i) * depth / fx
        y_cam = (cy - cy_i) * depth / fy
        z_cam = depth

        # Convert to world coordinates (simplified: add camera pose)
        x_world = x_cam + camera_pose.get('x', 0)
        y_world = y_cam + camera_pose.get('y', 0)
        z_world = z_cam + camera_pose.get('z', 0)

        return {
            'x': round(x_world, 3),
            'y': round(y_world, 3),
            'z': round(z_world, 3)
        }

    except Exception as e:
        raise RuntimeError(f"Depth estimation failed: {e}")


class DepthEstimator:
    """
    Multi-frame depth estimation using Structure from Motion (SfM)
    Branches off object detection to collect frames for depth processing
    """
    
    def __init__(self):
        self.frame_buffer = []  # Stores [frame1, frame2, frame3, frame4, frame5, frame10]
        self.successful_detection_count = 0  # Count of successful object detections
        self.movement_frames_collected = 0  # Count of movement frames (1-5)
        self.reference_frame_collected = False  # Whether we got frame 10
        
    def should_collect_frame(self, detection_count):
        """
        Check if this successful object detection should be collected for depth
        
        Args:
            detection_count: Current successful object detection number
            
        Returns:
            bool: True if this frame should be collected
        """
        # Collect frames 1-5 (movement frames)
        if detection_count <= DEPTH_FRAME_COUNT:
            return True
        
        # Collect frame 10 (reference frame after camera returns)
        if detection_count == REFERENCE_FRAME_POSITION:
            return True
            
        # Skip frames 6-9
        return False
        
    def collect_frame(self, frame: np.ndarray, detection_result: Dict, detection_count: int) -> Optional[Dict]:
        """
        Collect frames for multi-frame depth estimation (object detection branching)
        
        Args:
            frame: Current video frame
            detection_result: Object detection bounding box
            detection_count: Current successful detection number
            
        Returns:
            dict: Depth result when all frames collected, None otherwise
        """
        global DEPTH_PROCESSING_BUSY
        
        if DEPTH_PROCESSING_BUSY:
            return None  # Skip if busy - NO WAITING
        
        # Determine frame type
        is_movement_frame = detection_count <= DEPTH_FRAME_COUNT
        is_reference_frame = detection_count == REFERENCE_FRAME_POSITION
        
        # Add frame + detection data to buffer
        frame_data = {
            'frame': frame.copy(),
            'bounding_box': detection_result,
            'timestamp': datetime.now(),
            'detection_number': detection_count,
            'frame_type': 'movement' if is_movement_frame else 'reference'
        }
        self.frame_buffer.append(frame_data)
        
        # Update counters
        if is_movement_frame:
            self.movement_frames_collected += 1
            logger.debug(f"Movement frame collected: {self.movement_frames_collected}/{DEPTH_FRAME_COUNT}")
        elif is_reference_frame:
            self.reference_frame_collected = True
            logger.debug(f"Reference frame collected (detection #{detection_count})")
        
        # Process when we have all frames (5 movement + 1 reference)
        if self.movement_frames_collected == DEPTH_FRAME_COUNT and self.reference_frame_collected:
            logger.info(f"All frames collected ({len(self.frame_buffer)} total), starting depth processing")
            return self.process_depth_batch()
        
        return None
        
    def process_depth_batch(self) -> Optional[Dict]:
        """Process all collected frames using SfM (with flag protection)"""
        global DEPTH_PROCESSING_BUSY
        
        if DEPTH_PROCESSING_BUSY:
            return None  # Skip if busy
            
        DEPTH_PROCESSING_BUSY = True
        try:
            logger.info(f"Processing depth batch with {len(self.frame_buffer)} frames")
            
            # Use ALL 5 frames for SfM accuracy
            point_cloud = self._calculate_sfm_from_frames(self.frame_buffer)
            
            # Extract distance from LATEST frame's bounding box
            latest_frame = self.frame_buffer[-1]
            distance = self._extract_depth_at_bbox(point_cloud, latest_frame['bounding_box'])
            
            result = {
                'distance': distance,
                'reference_frame': 'latest',
                'confidence': self._calculate_confidence(),
                'method': 'sfm_multiframe',
                'frames_used': len(self.frame_buffer)
            }
            
            # Clear buffer for next batch
            self.reset_collection()
            logger.info(f"Depth processing complete: {distance:.2f}m")
            return result
            
        except Exception as e:
            logger.error(f"Depth batch processing failed: {e}")
            # Use fallback method on reference frame (should be last frame)
            reference_frame = None
            for frame_data in self.frame_buffer:
                if frame_data.get('frame_type') == 'reference':
                    reference_frame = frame_data
                    break
            
            if reference_frame is None:
                reference_frame = self.frame_buffer[-1]  # Use last frame as fallback
            
            fallback_result = self.estimate_distance_object_size(
                reference_frame['bounding_box'], 
                reference_frame['frame']
            )
            self.reset_collection()
            return fallback_result
            
        finally:
            DEPTH_PROCESSING_BUSY = False
    
    def reset_collection(self):
        """Reset the frame collection state"""
        self.frame_buffer.clear()
        self.successful_detection_count = 0
        self.movement_frames_collected = 0
        self.reference_frame_collected = False
        logger.debug("Depth collection reset")
    
    def get_collection_status(self):
        """Get current collection status for display"""
        return {
            'movement_frames_collected': self.movement_frames_collected,
            'movement_frames_needed': DEPTH_FRAME_COUNT,
            'reference_frame_collected': self.reference_frame_collected,
            'total_frames_collected': len(self.frame_buffer),
            'waiting_for_reference': (self.movement_frames_collected == DEPTH_FRAME_COUNT and not self.reference_frame_collected)
        }
        
    def _calculate_sfm_from_frames(self, frames: List[Dict]) -> np.ndarray:
        """
        Calculate Structure from Motion using collected frames
        Uses movement frames (1-5) and reference frame (10) for depth estimation
        
        Args:
            frames: List of frame data [movement_frames..., reference_frame]
            
        Returns:
            np.ndarray: Estimated depth map
        """
        try:
            # Separate movement frames from reference frame
            movement_frames = [f for f in frames if f.get('frame_type') == 'movement']
            reference_frames = [f for f in frames if f.get('frame_type') == 'reference']
            
            if len(movement_frames) < 2 or len(reference_frames) == 0:
                logger.warning("Insufficient frames for SfM, using object size method")
                return self._use_object_size_method(frames[-1])  # Use reference frame
            
            reference_frame = reference_frames[0]
            
            # Use first movement frame and reference frame for stereo estimation
            frame1 = movement_frames[0]['frame']
            frame2 = reference_frame['frame']
            
            # Extract features
            orb = cv2.ORB_create(nfeatures=1000)
            
            gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
            
            kp1, desc1 = orb.detectAndCompute(gray1, None)
            kp2, desc2 = orb.detectAndCompute(gray2, None)
            
            if desc1 is None or desc2 is None:
                logger.warning("No features found, using object size method")
                return self._use_object_size_method(reference_frame)
            
            # Match features
            matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = matcher.match(desc1, desc2)
            
            if len(matches) < 20:
                logger.warning(f"Too few matches ({len(matches)}), using object size method")
                return self._use_object_size_method(reference_frame)
            
            # Sort matches by distance
            matches = sorted(matches, key=lambda x: x.distance)
            good_matches = matches[:min(50, len(matches))]
            
            # Extract matched points
            pts1 = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            pts2 = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            
            # Calculate fundamental matrix and essential matrix
            F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 3.0, 0.99)
            
            if F is None:
                logger.warning("Could not calculate fundamental matrix, using object size method")
                return self._use_object_size_method(reference_frame)
            
            # Camera intrinsic parameters (you may need to calibrate these)
            K = np.array([[CAMERA_INTRINSICS['fx'], 0, CAMERA_INTRINSICS['cx']],
                         [0, CAMERA_INTRINSICS['fy'], CAMERA_INTRINSICS['cy']],
                         [0, 0, 1]], dtype=np.float32)
            
            # Calculate essential matrix
            E = K.T @ F @ K
            
            # Recover pose from essential matrix
            _, R, t, pose_mask = cv2.recoverPose(E, pts1, pts2, K)
            
            # Create projection matrices
            P1 = K @ np.hstack((np.eye(3), np.zeros((3, 1))))
            P2 = K @ np.hstack((R, t))
            
            # Triangulate points
            pts_4d = cv2.triangulatePoints(P1, P2, pts1[mask.ravel() == 1].reshape(-1, 1, 2).transpose(2, 0, 1),
                                          pts2[mask.ravel() == 1].reshape(-1, 1, 2).transpose(2, 0, 1))
            
            # Convert to 3D points
            pts_3d = (pts_4d[:3] / pts_4d[3]).T
            
            # Calculate median depth as estimate
            depths = np.abs(pts_3d[:, 2])  # Z coordinates
            median_depth = np.median(depths)
            
            # Clamp depth to reasonable range
            median_depth = max(0.3, min(median_depth, 15.0))
            
            logger.info(f"SfM calculated depth: {median_depth:.2f}m from {len(good_matches)} matches")
            
            # Create depth map with calculated depth
            height, width = reference_frame['frame'].shape[:2]
            depth_map = np.full((height, width), median_depth, dtype=np.float32)
            
            return depth_map
            
        except Exception as e:
            logger.warning(f"SfM calculation failed: {e}, using object size method")
            reference_frame = frames[-1] if frames else None
            if reference_frame:
                return self._use_object_size_method(reference_frame)
            else:
                # Ultimate fallback
                height, width = frames[0]['frame'].shape[:2]
                return np.full((height, width), 2.0, dtype=np.float32)
    
    def _use_object_size_method(self, frame_data: Dict) -> np.ndarray:
        """Create depth map using object size estimation method"""
        frame = frame_data['frame']
        detection = frame_data['bounding_box']
        
        # Calculate distance using object size
        distance = self._estimate_depth_from_bbox_size(detection)
        
        # Create depth map
        height, width = frame.shape[:2]
        depth_map = np.full((height, width), distance, dtype=np.float32)
        
        logger.info(f"Using object size method: {distance:.2f}m")
        return depth_map
        
    def _extract_depth_at_bbox(self, point_cloud: np.ndarray, bounding_box: Dict) -> float:
        """
        Extract depth at the center of the bounding box
        
        Args:
            point_cloud: Estimated depth map
            bounding_box: Object detection bounding box
            
        Returns:
            float: Estimated distance in meters
        """
        try:
            # Extract center of bounding box
            x1 = int(bounding_box['x'])
            y1 = int(bounding_box['y'])
            x2 = x1 + int(bounding_box['width'])
            y2 = y1 + int(bounding_box['height'])
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            # Ensure coordinates are within bounds
            height, width = point_cloud.shape
            cx = max(0, min(cx, width - 1))
            cy = max(0, min(cy, height - 1))
            
            # Get depth at center
            depth = float(point_cloud[cy, cx])
            
            if np.isnan(depth) or depth <= 0.0:
                logger.warning("Invalid depth from SfM, using fallback estimation")
                return self._estimate_depth_from_bbox_size(bounding_box)
            
            return depth
            
        except Exception as e:
            logger.warning(f"Depth extraction failed: {e}, using fallback")
            return self._estimate_depth_from_bbox_size(bounding_box)
        
    def _estimate_depth_from_bbox_size(self, bounding_box: Dict) -> float:
        """
        Fallback depth estimation based on bounding box size
        Assumes human target with average height of 1.7m
        """
        bbox_height = bounding_box['height']
        
        # Rough estimation: human height = 1.7m
        # Distance = (real_height * focal_length) / pixel_height
        real_height = 1.7  # meters
        focal_length = CAMERA_INTRINSICS['fy']
        
        if bbox_height > 0:
            distance = (real_height * focal_length) / bbox_height
            return max(0.5, min(distance, 10.0))  # Clamp between 0.5m and 10m
        else:
            return 2.0  # Default fallback
        
    def _calculate_confidence(self) -> float:
        """Calculate confidence score based on frame collection quality"""
        if len(self.frame_buffer) >= DEPTH_FRAME_COUNT:
            # Full batch processed
            return 0.85
        elif len(self.frame_buffer) >= 3:
            # Partial batch
            return 0.65
        else:
            # Fallback method used
            return 0.4
        
    def estimate_distance_object_size(self, detection: Dict, frame: np.ndarray) -> Dict:
        """
        Fallback: single frame object size method
        
        Args:
            detection: Object detection bounding box
            frame: Current video frame
            
        Returns:
            dict: Distance estimation result
        """
        try:
            distance = self._estimate_depth_from_bbox_size(detection)
            
            return {
                'distance': distance,
                'reference_frame': 'current',
                'confidence': 0.4,
                'method': 'object_size_fallback',
                'frames_used': 1
            }
            
        except Exception as e:
            logger.error(f"Fallback distance estimation failed: {e}")
            return {
                'distance': 2.0,  # Default fallback
                'reference_frame': 'current',
                'confidence': 0.1,
                'method': 'default_fallback',
                'frames_used': 1
            }
