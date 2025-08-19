import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from object_detect_node import object_detect_node
import cv2
import numpy as np
from PIL import Image

def test_yolo_on_image():
    image_path = os.path.join(os.path.dirname(__file__), "test_image.jpg")
    print(f"Loading image: {image_path}")
    
    # Use PIL to load image and convert to cv2 format to avoid ultralytics patching issues
    pil_image = Image.open(image_path)
    image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
    if image is None:
        print("Failed to load image. Make sure the path is correct.")
        return

    print("Running object detection for: 'chair'")
    result = object_detect_node("Go", "chair", image)

    if result:
        print("Detected object bounding box:", result)
        
        # Draw bounding box
        x, y, w, h = result["x"], result["y"], result["width"], result["height"]
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(image, "chair", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)

        # Show image
        cv2.imshow("Detection Result", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No matching object detected.")

if __name__ == "__main__":
    test_yolo_on_image()
