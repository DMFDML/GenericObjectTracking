# SiamMask.py

The `SiamMaskSharpTracker` class wrapper for the SiamMask Sharp tracking algorithm. It is designed for tracking 2D objects and provides methods to start, update, and draw bounding boxes around the tracked object.


## Usage

```python
from Trackers2D.SiamMask import SiamMaskSharpTracker

# Create an instance of the SiamMaskSharpTracker object tracker
siam_mask_tracker = SiamMaskSharpTracker()

# Start the object tracking process
siam_mask_tracker.start(image, box)

# Update the tracked object with a new image
updated_box = siam_mask_tracker.update(image)

# Draw the bounding box and mask around the tracked object on the input image
siam_mask_tracker.drawBox(img)

# Stop the object tracking process
siam_mask_tracker.stopTracking()
```

## Description

The `SiamMaskSharpTracker` class is an object tracker designed for 2D object tracking using the SiamMask Sharp tracking algorithm. It utilizes a pre-trained SiamMask model with sharpness-enhancing features and provides methods to start, update, draw bounding boxes with masks, and stop the object tracking process.

## Class Attributes

- `device`: A string representing the device to use for tracking (e.g., 'cuda' for GPU or 'cpu' for CPU).
- `config`: A dictionary representing the configurations used for the SiamMask tracking.
- `siammask`: An instance of the `Custom` class representing the SiamMask model for tracking.

## Methods

- `start(self, image, box)`: Starts the object tracking process with an initial image and bounding box.
- `update(self, image)`: Updates the tracked object with a new image and returns the updated bounding box.
- `stopTracking(self)`: Stops the object tracking process (not implemented).
- `getMask(self)`: Returns the mask of the tracked object.
- `drawBox(self, img)`: Draws the bounding box and mask around the tracked object on the input image.
