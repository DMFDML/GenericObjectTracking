import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
import sys
import os
sys.path.append(os.getcwd() + './re3-tensorflow')
from src.Trackers.TrackerInterface import Tracker
from tracker import re3_tracker
from re3_utils.util import drawing
from re3_utils.util import bb_util
from re3_utils.util import im_util
import os
import numpy as np
import cv2


from constants import GPU_ID
from objectTrackingConstants import PRE_TRAINED_MODEL_Re3



class Re3Tracker(Tracker):

    def __init__(self):
        self.model = os.path.join(os.path.dirname(__file__), PRE_TRAINED_MODEL_Re3)
        self.tracker = re3_tracker.Re3Tracker(GPU_ID, self.model)


    def start(self, image, box):
        re_box = np.array([box[0], box[1], box[0]+box[2], box[1]+box[3]])
        print(box, " ", re_box)
        
        self.tracker.track('webcam', image, re_box)

    def update(self, image):
        box = self.tracker.track('webcam', image)
        x,y,x2,y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        self.box = min(max(x, 0), image.shape[1]), min(max(y, 0), image.shape[0]), max(min(x2, image.shape[1])-x, 0), max(min(y2, image.shape[0])-y, 0)
        return self.box
    
    def stopTracking(self):
        pass

    def getMask(self):
        return np.array([])
    
    def drawBox(self, img):
        if (not self.box == []):
            x,y,w,h = self.box
            cv2.rectangle(img, (x,y), ((x+w), (y+h)), (0, 255, 0), 3, 1)
            cv2.putText(img, "Got", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(img, "Lost", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
