import cv2
import numpy as np
import os
import logging

logger = logging.getLogger(__name__)

class SkeletonYolo:
    DEFAULT_CLASSES = [
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

    def __init__(self, classesFile='obstacles.names', modelConfiguration='yolov3.cfg', modelWeights='yolov3.weights',
                 whT=320, confThreshold=0.5, nmsThreshold=0.3, classes=None):
        self.whT = whT
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold

        if classes is not None:
            self.classNames = [str(c).strip() for c in classes if str(c).strip()]
        elif os.path.exists(classesFile):
            with open(classesFile, 'rt') as f:
                self.classNames = [line.strip() for line in f if line.strip()]
        else:
            self.classNames = self.DEFAULT_CLASSES.copy()
            logger.warning(
                'YOLO class list file not found (%s). Falling back to default COCO class names.', classesFile)

        self.net = None
        if os.path.exists(modelConfiguration) and os.path.exists(modelWeights):
            try:
                self.net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            except cv2.error as exc:
                logger.warning('Failed to load YOLO network: %s', exc)
                self.net = None
        else:
            logger.warning(
                'YOLO config or weights missing. Expected %s and %s.', modelConfiguration, modelWeights)

    def findObjects(self, img):
        if self.net is None:
            return []

        if img is None or img.size == 0:
            return []

        if img.ndim == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        blob = cv2.dnn.blobFromImage(img, 1 / 255, (self.whT, self.whT), [0, 0, 0], 1, crop=False)
        self.net.setInput(blob)

        layerNames = self.net.getLayerNames()
        outputNames = [layerNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(outputNames)

        hT, wT = img.shape[:2]
        bbox = []
        classIds = []
        confs = []

        for output in outputs:
            for det in output:
                scores = det[5:]
                classId = int(np.argmax(scores))
                confidence = float(scores[classId])
                if confidence > self.confThreshold:
                    w, h = int(det[2] * wT), int(det[3] * hT)
                    x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(confidence)

        if not bbox:
            return []

        indices = cv2.dnn.NMSBoxes(bbox, confs, self.confThreshold, self.nmsThreshold)
        detected_objects = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = bbox[i]
                cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
                label = self.classNames[classIds[i]].upper() if classIds[i] < len(self.classNames) else 'UNKNOWN'
                cv2.putText(img, f'{label} {int(confs[i] * 100)}%',
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                detected_objects.append(label)
        return detected_objects