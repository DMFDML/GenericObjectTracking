import sys
import os
sys.path.append(os.getcwd() + '/SiamMask')
sys.path.append(os.getcwd() + '/SiamMask/experiments/siammask_sharp')
from Trackers2D.TrackerInterface import Tracker
from tools.test import *
from custom import Custom
from models.siammask_sharp import SiamMask
import json
from utils.tracker_config import TrackerConfig
import torch
import torch.nn as nn
import cv2
from Util.objectTrackingConstants import CONFIG, PRE_TRAINED_MODEL_SiamMask



class SiamMaskSharpTracker(Tracker):

    def __init__(self):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        torch.backends.cudnn.benchmark = True

        # Load Configs
        self.config = json.load(open(CONFIG))

        self.siammask = Custom(anchors=self.config['anchors'])
        self.siammask = load_pretrain(self.siammask, PRE_TRAINED_MODEL_SiamMask)
        self.siammask.eval().to(self.device)


    def start(self, image, box):
        target_pos = np.array([box[0] + box[2] / 2, box[1] + box[3] / 2])
        target_sz = np.array([box[2], box[3]])

        self.state = siamese_init(image, target_pos, target_sz, self.siammask, self.config['hp'], device=self.device)  # init tracker

    def update(self, image):
        self.state = siamese_track(self.state, image, mask_enable=True, refine_enable=True, device=self.device)  # track
        location = cxy_wh_2_rect(self.state['target_pos'], self.state['target_sz'])
        x,y,w,h = int(location[0]), int(location[1]), int(location[2]), int(location[3])
        self.box = min(max(x, 0), image.shape[1]), min(max(y, 0), image.shape[0]), max(min(x+w, image.shape[1])-x, 0), max(min(y+h, image.shape[0])-y, 0)
        return self.box
    
    def stopTracking(self):
        pass
    
    def getMask(self):
        return self.state['mask'] > self.state['p'].seg_thr
    
    def drawBox(self, img):
        if (not self.box == []):
            x,y,w,h = int(self.box[0]), int(self.box[1]), int(self.box[2]), int(self.box[3])
            cv2.rectangle(img, (x,y), ((x+w), (y+h)), (0, 255, 0), 3, 1)
            mask = self.getMask()
            img[:, :, 2] = (mask > 0) * 255 + (mask == 0) * img[:, :, 2]
            cv2.putText(img, "Got", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(img, "Lost", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)