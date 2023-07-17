from abc import ABC, abstractmethod

class Tracker(ABC):
    
    @abstractmethod
    def start(self, image, box):
        # start the tracker with the first frame and the bounding box of the object being tracked
        # Input: image, The frame of the video being processed
        #        box, The bounding box of the object being tracked
        # Output: None
        pass

    @abstractmethod
    def update(self, image):
        # update the tracker with a new frame in order to generate a new bounding box
        # Input: img, The frame of the video being processed
        # Output: The bounding box of the object being tracked
        pass

    @abstractmethod
    def stopTracking(self):
        # stop tracking the object
        # Input: None
        # Output: None
        #
        pass

    @abstractmethod
    def getMask(self):
        # Return if the tracer has a mask or not
        # Input: None
        # Output: None
        #
        pass

    @abstractmethod
    def drawBox(self, img):
        # Draw the bounding box on the image
        # Input: image
        # Output: None
        #
        pass
