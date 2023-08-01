# Tracker Abstract Class

The `Tracker` abstract class defines an interface for object trackers. It provides abstract methods that need to be implemented by any concrete tracker class that inherits from it. The class defines methods to start tracking, update the tracked object with new frames, stop tracking, get the mask of the tracked object, and draw the bounding box on an image.

## Usage

```python
from Trackers2D.TrackerInterface import Tracker

class ExampleTracker(Tracker):

    def start(self, image, box):
        # start the tracker with the first frame and the bounding box of the object being tracked

    @abstractmethod
    def update(self, image):
        # update the tracker with a new frame in order to generate a new bounding box

    @abstractmethod
    def stopTracking(self):
        # stop tracking the object

    @abstractmethod
    def getMask(self):
        # Return if the tracer has a mask or not

    @abstractmethod
    def drawBox(self, img):
        # Draw the bounding box on the image
```

## Description

The `Tracker` abstract class defines an interface for object trackers. It is an abstract class that provides a set of methods that need to be implemented by any concrete tracker class that inherits from it. The methods include starting the tracker with the first frame and bounding box of the object being tracked, updating the tracker with a new frame to generate a new bounding box, stopping tracking, getting the mask of the tracked object, and drawing the bounding box on an image.

## Methods

- `start(self, image, box)`: Starts the object tracking process with an initial image and bounding box.
- `update(self, image)`: Updates the tracked object with a new image and returns the updated bounding box.
- `stopTracking(self)`: Stops the object tracking process.
- `getMask(self)`: Returns if the tracker has a mask or not.
- `drawBox(self, img)`: Draws the bounding box on the input image.
