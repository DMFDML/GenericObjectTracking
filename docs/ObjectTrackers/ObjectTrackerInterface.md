# ObjectTrackerInterface.py

The `ObjectTracker` class is an abstract class that defines the interface for implementing object tracking functionality. It provides two abstract methods `trackFrame` and `startTracking`, which must be implemented by any subclass.



## Usage

```python
from abc import ABC, abstractmethod

# Example implementation of the ObjectTracker class
class ExampleObjectTracker(ObjectTracker):

    def trackFrame(self, img):
        # Your implementation of object tracking in a single frame
        # Calculate rotation and translation of the tracked object in the frame
        # Return rotation and translation

    def startTracking(self):
        # Your implementation of starting the object tracking process
        # This method will use the trackFrame method to track the object in real-time
        pass
```

## Description

The `ObjectTracker` class is an abstract base class that provides an interface for implementing object tracking functionality. Any class that inherits from this abstract class must implement the `trackFrame` and `startTracking` methods.

## Methods

- `trackFrame(self, img)`: This abstract method is responsible for tracking the object in a single frame. It takes the input frame `img` and returns the estimated rotation and translation of the tracked object.

- `startTracking(self)`: This abstract method is responsible for starting the object tracking process. It should use the `trackFrame` method to track the object in real-time.

## Note

- The `ObjectTracker` class is an abstract class and cannot be instantiated directly. It serves as a blueprint for creating concrete object tracker classes.
- Any class that inherits from the `ObjectTracker` class must implement the `trackFrame` and `startTracking` methods with their specific implementations for object tracking.

