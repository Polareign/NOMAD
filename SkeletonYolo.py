import cv2
import numpy as np

class SkeletonYolo:
    def __init__(self, classesFile='obstacles.names', modelConfiguration='yolov3.cfg', modelWeights='yolov3.weights',
                 whT=320, confThreshold=0.5, nmsThreshold=0.3):
        self.whT = whT
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold

        self.classNames = []
        with open(classesFile, 'rt') as f:
            self.classNames = f.read().rstrip('\n').split('\n')

        self.net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def findObjects(self, img):
        # Convert to BGR if needed (picamera2 might give RGB)
        if img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        blob = cv2.dnn.blobFromImage(img, 1/255, (self.whT, self.whT), [0, 0, 0], 1, crop=False)
        self.net.setInput(blob)
        layerNames = self.net.getLayerNames()
        outputNames = [layerNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(outputNames)

        hT, wT, cT = img.shape
        bbox = []
        classIds = []
        confs = []

        for output in outputs:
            for det in output:
                scores = det[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    w, h = int(det[2] * wT), int(det[3] * hT)
                    x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(float(confidence))

        indices = cv2.dnn.NMSBoxes(bbox, confs, self.confThreshold, self.nmsThreshold)
        detected_objects = []
        for i in indices:
            i = i[0]
            box = bbox[i]
            x, y, w, h = box[0], box[1], box[2], box[3]
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            cv2.putText(img, f'{self.classNames[classIds[i]].upper()} {int(confs[i] * 100)}%',
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            detected_objects.append(self.classNames[classIds[i]].upper())
        return detected_objects