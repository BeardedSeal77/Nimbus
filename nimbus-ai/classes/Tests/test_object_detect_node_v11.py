import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from object_detect_node_v11 import ObjectDetector

def test_yolo_v11_on_image():
    image_path = os.path.join(os.path.dirname(__file__), "test_image.jpg")
    print(f"Testing YOLOv11 on image: {image_path}")

    # Create detector instance
    detector = ObjectDetector()
    
    print("Running YOLOv11 object detection for: 'chair'")
    result = detector.detect("chair", image_path)

    if result:
        print("✓ Detected object!")
        print(f"Bounding box: {result}")
        
        # Show side-by-side comparison
        print("Displaying side-by-side comparison...")
        detector.show_side_by_side(image_path, "chair")
            
    else:
        print("✗ No matching object detected.")
        # Still show side-by-side for all detections
        print("Showing side-by-side comparison...")
        detector.show_side_by_side(image_path)

if __name__ == "__main__":
    test_yolo_v11_on_image()
