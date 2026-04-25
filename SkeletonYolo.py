import cv2
import logging
from ultralytics import YOLO

logger = logging.getLogger(__name__)

class SkeletonYolo:
    def __init__(self, confThreshold=0.5, classes=None):
        """
        Initialize YOLO11n model for object detection.
        
        Args:
            confThreshold: Confidence threshold for detections (default 0.5)
            classes: List of obstacle classes to detect (optional, uses YOLO defaults)
        """
        self.confThreshold = confThreshold
        self.model = None
        self.classNames = classes if classes is not None else self._get_default_coco_classes()
        
        try:
            # Load YOLO11n model (automatically downloads if not present)
            self.model = YOLO('yolo11n.pt')
            self.model.conf = confThreshold
            logger.info('YOLO11n model loaded successfully')
        except Exception as exc:
            logger.error('Failed to load YOLO11n model: %s', exc)
            self.model = None

    @staticmethod
    def _get_default_coco_classes():
        """Return default COCO class names."""
        return [
            'person', 'bicycle', 'car', 'motorbike', 'aeroplane', 'bus', 'train', 'truck', 'boat',
            'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
            'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
            'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'sofa', 'pottedplant', 'bed', 'diningtable', 'toilet', 'tvmonitor', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
            'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def findObjects(self, img):
        """
        Detect objects in image using YOLO11n.
        
        Args:
            img: Input image (BGR or RGB)
            
        Returns:
            List of detected object class names as strings
        """
        if self.model is None or img is None or img.size == 0:
            return []

        try:
            # Run inference
            results = self.model(img, verbose=False)
            
            detected_objects = []
            
            # Process detections
            for result in results:
                if result.boxes is not None and len(result.boxes) > 0:
                    for box in result.boxes:
                        cls_id = int(box.cls[0])
                        conf = float(box.conf[0])
                        
                        # Filter by confidence threshold
                        if conf >= self.confThreshold:
                            # Get class name
                            class_name = result.names.get(cls_id, f'class_{cls_id}')
                            detected_objects.append(class_name)
                            
                            # Draw bounding box and label on image
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 2)
                            label = f'{class_name} {int(conf * 100)}%'
                            cv2.putText(img, label, (x1, y1 - 10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            return detected_objects
            
        except Exception as exc:
            logger.error('Error during inference: %s', exc)
            return []