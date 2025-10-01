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

class ObjectDetector:
    def __init__(self):
        """Initialize YOLO model with GPU optimization for RTX 4070 Super"""
        import torch

        # Use YOLOv11n for speed (change to yolo11s/m/l for accuracy)
        self.model = YOLO("yolo11n.pt")

        # Check CUDA availability
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if self.device == 'cuda':
            print(f"GPU detected: {torch.cuda.get_device_name(0)}")
            print(f"CUDA version: {torch.version.cuda}")
        else:
            print("WARNING: Running on CPU - GPU not detected!")

        # Skip warmup - will happen on first real frame
        self.last_results = None
        print(f"YOLO initialized on {self.device} with FP16 acceleration")
        print("Note: First inference will be slower due to GPU warmup")

    def detect(self, target_object: str, image_source) -> dict:
        """
        Detects the specified object in the given image using YOLO.

        Parameters:
            target_object (str): Name of the object to detect (e.g. "chair")
            image_source: Image path, URL, PIL image, or numpy array

        Returns:
            dict: Bounding box in format {'x': x, 'y': y, 'width': w, 'height': h}
                  or {} if not found
        """
        try:
            # Run detection with GPU and FP16 for max speed
            self.last_results = self.model.predict(
                image_source,
                device=self.device,
                verbose=False,
                half=True,  # FP16 inference on GPU
                imgsz=640   # Input size (can reduce to 416 for more speed)
            )

            for result in self.last_results:
                boxes = result.boxes
                names = result.names  

                if boxes is not None:
                    for box in boxes:
                        cls_id = int(box.cls[0])
                        label = names[cls_id].lower()
                        
                        if target_object.lower() in label or label in target_object.lower():
                            xyxy = box.xyxy[0].cpu().numpy()
                            x1, y1, x2, y2 = map(int, xyxy)

                            return {
                                'x': x1,
                                'y': y1,
                                'width': x2 - x1,
                                'height': y2 - y1
                            }

            # No matching object
            return {}

        except Exception as e:
            raise RuntimeError(f"Object detection failed: {e}")
    
    def show_results(self):
        """
        Display the last detection results with bounding boxes.
        Must call detect() first.
        """
        if self.last_results is None:
            print("No results to show. Call detect() first.")
            return
        
        for result in self.last_results:
            result.show()
    
    def show_side_by_side(self, image_source, target_object: str = None):
        """
        Display original image and bounding box overlay side by side.
        
        Parameters:
            image_source: Image path or source
            target_object: Optional - highlight only this object type
        """
        if self.last_results is None:
            print("No results to show. Call detect() first.")
            return
        
        import matplotlib.pyplot as plt
        
        # Load original image
        original_img = compatible_imread(image_source)
        if original_img is None:
            print("Could not load image")
            return
        
        # Convert BGR to RGB for matplotlib
        original_img_rgb = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
        
        # Create overlay image (black background with just bounding boxes)
        overlay_img = np.zeros_like(original_img_rgb)
        
        # Draw bounding boxes on overlay
        for result in self.last_results:
            if result.boxes is not None:
                for box in result.boxes:
                    # Get box coordinates
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)
                    
                    # Get class name
                    cls_id = int(box.cls[0])
                    label = result.names[cls_id]
                    conf = float(box.conf[0])
                    
                    # Color selection - highlight target object in green, others in red
                    if target_object and target_object.lower() in label.lower():
                        color = (0, 255, 0)  # Green for target object
                        thickness = 3
                    else:
                        color = (255, 0, 0)  # Red for other objects
                        thickness = 2
                    
                    # Draw rectangle
                    cv2.rectangle(overlay_img, (x1, y1), (x2, y2), color, thickness)
                    
                    # Add label
                    label_text = f"{label}: {conf:.2f}"
                    cv2.putText(overlay_img, label_text, (x1, y1-10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness-1)
        
        # Display side by side
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
        
        ax1.imshow(original_img_rgb)
        ax1.set_title("Original Image")
        ax1.axis('off')
        
        ax2.imshow(overlay_img)
        ax2.set_title("Bounding Boxes Only")
        ax2.axis('off')
        
        plt.tight_layout()
        plt.show()

# Convenience function for backward compatibility
def object_detect_node(target_object: str, image_source) -> dict:
    """
    Convenience function that creates a detector and runs detection.
    For more control, use the ObjectDetector class directly.
    """
    detector = ObjectDetector()
    return detector.detect(target_object, image_source)