import numpy as np
from nimbus.AI.classes.depth_node import depth_node

def generate_dummy_depth_map(width=640, height=480, depth_value=2.0):
    """
    Generate a dummy depth map with a constant depth (in meters).
    """
    depth_map = np.full((height, width), depth_value, dtype=np.float32)
    return depth_map

def test_depth_estimation():
    # Simulated inputs
    bounding_box = {
        'x': 300,
        'y': 220,
        'width': 40,
        'height': 40
    }

    camera_pose = {
        'x': 1.0,
        'y': 2.0,
        'z': 0.5
    }

    # Generate dummy depth map with all pixels = 2.0m
    depth_map = generate_dummy_depth_map()

    # Dummy RGB frame (unused, but passed)
    rgb_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    print("🧪 Running depth estimation test...")
    try:
        object_position = depth_node(
            intent="go",
            bounding_box=bounding_box,
            rgb_frame=rgb_frame,
            camera_pose=camera_pose,
            depth_map=depth_map
        )

        print("✅ Object 3D position (world):", object_position)

    except Exception as e:
        print("❌ Test failed:", e)

if __name__ == "__main__":
    test_depth_estimation()
