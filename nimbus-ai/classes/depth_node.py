# use: OpenCV, RTAB-Map
# inputs: intent, bounding_box, video, camera ProcessLookupError
# output: detected object (object pose)

import numpy as np

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
