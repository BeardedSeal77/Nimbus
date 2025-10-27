import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'nimbus-ai'))

from classes.object_position_node import calculate_object_position_once

drone_pos = {'x': -5.854373358452609, 'y': 2.650245924141279, 'z': 2.030037776202643}
drone_yaw = 2.993544344880504

car_actual = {'x': -41.5139, 'y': 4.34169, 'z': 0.31}

actual_dx = car_actual['x'] - drone_pos['x']
actual_dy = car_actual['y'] - drone_pos['y']
actual_dz = car_actual['z'] - drone_pos['z']
actual_distance = math.sqrt(actual_dx**2 + actual_dy**2 + actual_dz**2)

print("=" * 80)
print("MANUAL CALCULATION TEST")
print("=" * 80)

print("\nDRONE STATE:")
print(f"  Position: ({drone_pos['x']:.2f}, {drone_pos['y']:.2f}, {drone_pos['z']:.2f})")
print(f"  Yaw: {drone_yaw:.4f} rad = {math.degrees(drone_yaw):.2f} deg")

print("\nCAR ACTUAL POSITION:")
print(f"  Position: ({car_actual['x']:.2f}, {car_actual['y']:.2f}, {car_actual['z']:.2f})")

print("\nACTUAL WORLD DELTAS (car - drone):")
print(f"  dx: {actual_dx:.3f} m")
print(f"  dy: {actual_dy:.3f} m")
print(f"  dz: {actual_dz:.3f} m")
print(f"  3D distance: {actual_distance:.3f} m")

actual_angle_in_world = math.atan2(actual_dy, actual_dx)
print(f"  Angle in world frame: {math.degrees(actual_angle_in_world):.2f} deg")

relative_angle = actual_angle_in_world - drone_yaw
relative_angle = ((relative_angle + math.pi) % (2 * math.pi)) - math.pi
print(f"  Angle relative to drone forward: {math.degrees(relative_angle):.2f} deg")

horizontal_dist = math.sqrt(actual_dx**2 + actual_dy**2)
print(f"  Horizontal distance: {horizontal_dist:.3f} m")

print("\nWhat the bounding box SHOULD be for the car to appear in frame:")
camera_fov = 0.7854
img_width = 1280
img_height = 720

vertical_angle = math.atan2(actual_dz, horizontal_dist)
print(f"  Vertical angle: {math.degrees(vertical_angle):.2f} deg")

pixel_offset_x = relative_angle / (camera_fov / 2.0)
bbox_center_x = (pixel_offset_x * (img_width / 2.0)) + (img_width / 2.0)

pixel_offset_y = vertical_angle / (camera_fov / 2.0)
bbox_center_y = (img_height / 2.0) - (pixel_offset_y * (img_height / 2.0))

print(f"  Expected bbox center: ({bbox_center_x:.0f}, {bbox_center_y:.0f})")

if 0 <= bbox_center_x <= img_width and 0 <= bbox_center_y <= img_height:
    print("  Car IS in frame")

    test_bbox = {
        'x': bbox_center_x - 50,
        'y': bbox_center_y - 50,
        'width': 100,
        'height': 100
    }

    camera_config = {
        'fov': camera_fov,
        'width': img_width,
        'height': img_height
    }

    print("\nTEST: Calculating position with perfect bbox:")
    calc_pos = calculate_object_position_once(
        test_bbox,
        actual_distance,
        drone_pos,
        drone_yaw,
        camera_config
    )

    if calc_pos:
        print(f"  Calculated: ({calc_pos['x']:.3f}, {calc_pos['y']:.3f}, {calc_pos['z']:.3f})")
        error_dx = car_actual['x'] - calc_pos['x']
        error_dy = car_actual['y'] - calc_pos['y']
        error_dz = car_actual['z'] - calc_pos['z']
        error_total = math.sqrt(error_dx**2 + error_dy**2 + error_dz**2)
        print(f"  Error: {error_total:.3f} m (dx={error_dx:.3f}, dy={error_dy:.3f}, dz={error_dz:.3f})")
else:
    print("  Car NOT in frame")

print("\n" + "=" * 80)
