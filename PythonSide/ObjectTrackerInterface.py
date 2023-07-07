from abc import ABC, abstractmethod
import json
import socket

class ObjectTracker(ABC):
    
    # Sends data to unity
    def sendData(self, rotation, centre):

        data = {
            "box": {
                "rotation": rotation.tolist(),
                "centre": centre.tolist(),
            },
        }

        packet = json.dumps(data, indent=3)
        self.sock.sendto( (packet).encode(), (self.ip, self.port) )

    @abstractmethod
    def startTracking(self):
        # start tracking the object
        # Input: image, The frame of the video being processed
        #        box, The bounding box of the object being tracked
        # Output: None
        pass

    @abstractmethod
    def benchMark(self, speed_file, translation_rotation):
        # benchmark the tracker
        # Input: speed_file, The open file for frame rate data
        #        translation_rotation, The open file for tranlation and rotation data
        # Output: None
        pass