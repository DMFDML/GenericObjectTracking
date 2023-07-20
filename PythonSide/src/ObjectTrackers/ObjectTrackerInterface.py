from abc import ABC, abstractmethod

from math import pi, atan2, sqrt
import numpy as np

class ObjectTracker(ABC):
    
    
    @abstractmethod
    def trackFrame(self, img):
        # start tracking the object
        # Input: img, The frame of the video being processed
        # Output: rotation, The rotation of the object
        #         translation, The translation of the object
        pass

    @abstractmethod
    def startTracking(self):
        # start tracking the object
        # Input: None
        # Output: None
        pass

