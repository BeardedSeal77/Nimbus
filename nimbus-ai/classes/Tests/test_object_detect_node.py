import cv2
from nimbus-AI.classes.object_detect_node import object_detect_node

def test_yolo_on_image():
    image_path = "nimbus/AI/classes/Tests/test_image.jpg"
    print(f"Loading image: {image_path}")
    
    image = cv2.imread(image_path)
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
