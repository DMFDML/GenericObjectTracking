# Re3Tracker Class

The `Re3Tracker` class is a wrapper for the RE3 (Real-time Efficient Edge Detection) tracking algorithm. It is designed for tracking 2D objects and provides methods to start, update, and draw bounding boxes around the tracked object.

## Usage

```python
from Trackers2D.Re3Tracker import Re3Tracker

# Create an instance of the Re3Tracker object tracker
re3_tracker = Re3Tracker()

# Start the object tracking process
re3_tracker.start(image, box)

# Update the tracked object with a new image
updated_box = re3_tracker.update(image)

# Draw the bounding box around the tracked object
re3_tracker.drawBox(img)

# Stop the object tracking process
re3_tracker.stopTracking()
```

## Description

The `Re3Tracker` class is an object tracker designed for 2D object tracking using the RE3 (Real-time Efficient Edge Detection) tracking algorithm. It utilizes a pre-trained model for efficient edge detection and provides methods to start, update, draw bounding boxes, and stop the object tracking process.

## Class Attributes

- `model`: A string representing the path to the pre-trained RE3 model.
- `tracker`: An instance of the `re3_tracker.Re3Tracker` class used for object tracking.

## Methods

- `start(self, image, box)`: Starts the object tracking process with an initial image and bounding box.
- `update(self, image)`: Updates the tracked object with a new image and returns the updated bounding box.
- `stopTracking(self)`: Stops the object tracking process (not implemented).
- `getMask(self)`: Returns an empty array as a mask (not implemented).
- `drawBox(self, img)`: Draws the bounding box around the tracked object on the input image.

