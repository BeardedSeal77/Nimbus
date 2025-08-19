import os
import cv2
import numpy as np

# Save original cv2.imread before ultralytics patches it
original_cv2_imread = cv2.imread

# Patch imread to ensure proper numpy array format for PyTorch compatibility  
def compatible_imread(filename, flags=cv2.IMREAD_COLOR):
    img = original_cv2_imread(filename, flags)
    if img is not None:
        # Convert to proper numpy array format for PyTorch
        return np.array(img, dtype=np.uint8)
    return img

# Apply patch before importing ultralytics
import ultralytics.utils.patches
ultralytics.utils.patches.imread = compatible_imread

from ultralytics import YOLO

# Get path to test image
image_path = os.path.join(os.path.dirname(__file__), "test_image.jpg")
print(f"Testing YOLO on: {image_path}")

# Load a model
model = YOLO("yolov8s.pt")  # load YOLOv8 small model

# Predict with the model
results = model(image_path)  # predict on local image

# Access the results
for result in results:
    if result.boxes is not None:
        print(f"Detected {len(result.boxes)} objects:")
        names = [result.names[cls.item()] for cls in result.boxes.cls.int()]
        confs = result.boxes.conf
        for i, name in enumerate(names):
            print(f"  {name}: confidence {confs[i]:.2f}")
        
        # Show the result
        result.show()
    else:
        print("No objects detected")