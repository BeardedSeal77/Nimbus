import math
import logging

logger = logging.getLogger(__name__)

TARGET_OBJECT_HEIGHT = 0.6

def calculate_object_position_once(bbox, distance, drone_position, drone_yaw, camera_config):
    """
    Calculate 3D world position from bounding box and distance using polar to cartesian conversion.

    This function converts:
    1. Bounding box (pixels) -> angular offset from drone forward axis
    2. (distance, angle) polar coordinates -> drone-relative cartesian (forward, lateral)
    3. Drone-relative coordinates -> world coordinates (home = 0,0,0)

    The object height is projected to 600mm (0.6m) above ground for consistent positioning.

    Args:
        bbox (dict): Bounding box with keys {'x', 'y', 'width', 'height'} in pixels
        distance (float): 3D distance to object in meters from depth detection
        drone_position (dict): Drone position {'x', 'y', 'z'} in meters
        drone_yaw (float): Drone yaw angle in radians
        camera_config (dict): Camera configuration {'fov', 'width', 'height'}

    Returns:
        dict: Object position {'x', 'y', 'z'} in world coordinates (meters)
              Returns None if calculation fails

    Example:
        bbox = {'x': 500, 'y': 300, 'width': 200, 'height': 150}
        distance = 5.0  # meters
        drone_pos = {'x': 1.0, 'y': 2.0, 'z': 2.0}
        drone_yaw = 0.5  # radians
        camera_cfg = {'fov': 0.7854, 'width': 1280, 'height': 720}

        position = calculate_object_position_once(bbox, distance, drone_pos, drone_yaw, camera_cfg)
        # position = {'x': 4.2, 'y': 3.1, 'z': 0.6}
    """
    try:
        fov = camera_config['fov']
        img_width = camera_config['width']
        img_height = camera_config['height']

        img_center_x = img_width / 2.0
        img_center_y = img_height / 2.0

        bbox_center_x = bbox['x'] + bbox['width'] / 2.0
        bbox_center_y = bbox['y'] + bbox['height'] / 2.0

        pixel_offset_x = (bbox_center_x - img_center_x) / img_center_x
        horizontal_angle = pixel_offset_x * (fov / 2.0)

        pixel_offset_y = (img_center_y - bbox_center_y) / img_center_y
        vertical_angle = pixel_offset_y * (fov / 2.0)

        logger.debug(f"Pixel offsets: x={pixel_offset_x:.3f}, y={pixel_offset_y:.3f}")
        logger.debug(f"Angular offsets: horiz={math.degrees(horizontal_angle):.2f}deg, vert={math.degrees(vertical_angle):.2f}deg")

        horizontal_distance = distance * math.cos(vertical_angle)
        vertical_distance = distance * math.sin(vertical_angle)

        object_z_raw = drone_position['z'] + vertical_distance

        logger.debug(f"Initial calc: horiz_dist={horizontal_distance:.2f}m, vert_dist={vertical_distance:.2f}m, z_raw={object_z_raw:.2f}m")

        if abs(object_z_raw - TARGET_OBJECT_HEIGHT) > 0.01:
            delta_z = TARGET_OBJECT_HEIGHT - drone_position['z']
            if abs(vertical_distance) > 0.01:
                scale_factor = delta_z / vertical_distance
                horizontal_distance = horizontal_distance * scale_factor
                logger.debug(f"Adjusted for z={TARGET_OBJECT_HEIGHT}m: scale={scale_factor:.3f}, new_horiz={horizontal_distance:.2f}m")

        forward_delta = horizontal_distance * math.cos(horizontal_angle)
        lateral_delta = horizontal_distance * math.sin(horizontal_angle)

        logger.debug(f"Drone-relative: forward={forward_delta:.2f}m, lateral={lateral_delta:.2f}m")

        world_dx = forward_delta * math.cos(drone_yaw) - lateral_delta * math.sin(drone_yaw)
        world_dy = forward_delta * math.sin(drone_yaw) + lateral_delta * math.cos(drone_yaw)

        object_world_x = drone_position['x'] + world_dx
        object_world_y = drone_position['y'] + world_dy
        object_world_z = TARGET_OBJECT_HEIGHT

        result = {
            'x': round(object_world_x, 3),
            'y': round(object_world_y, 3),
            'z': round(object_world_z, 3)
        }

        logger.info(f"Object position calculated: {result} (from bbox center ({bbox_center_x:.0f}, {bbox_center_y:.0f}), dist={distance:.2f}m)")

        return result

    except Exception as e:
        logger.error(f"Object position calculation failed: {e}", exc_info=True)
        return None
