# plotData.py

The script reads CSV files containing object tracking data and generates various graphs to visualize the performance of different object trackers. The script is organized into several functions, each outputing a different graph.

## Script Overview

The script is designed to perform the following tasks:

1. Read benchmark data from CSV files.
2. Calculate various metrics based on the data.
3. Generate graphs to visualize the performance of object trackers.

## Script Usage

To use the script, follow these steps:

1. Place the CSV files containing benchmark data in the appropriate folder (`.\\benchmark\\results\\`).

```bash
python ./benchmark/plotdata.py
```

## Script Description

The script begins by importing necessary modules and setting the font size for the generated plots. It then defines several functions to plot different aspects of the benchmark data. Let's go through each function:

### `getValues(data)`

This function calculates the average speed, accuracy of rotation, and accuracy of translation for each object and tracker. It writes these statistics to a CSV file named `values.csv` in the `.\\benchmark\\results\\graphsAndValues\\` folder.

### `plotSpeed(data)`

This function generates a 2x2 grid of line plots, with each subplot representing the frame vs. time (speed) for a different object (coin, cylinder, drill, vive) and various trackers. The plots are saved as `frame_vs_time_all.png` in the `.\\benchmark\\results\\graphsAndValues\\` folder.

### `plotRotation(data)`

This function generates a 2x2 grid of line plots, with each subplot representing the frame vs. rotation (in radians) for a different object and various trackers. The plots are saved as `frame_vs_rotation_all.png` in the `.\\benchmark\\results\\graphsAndValues\\` folder.

### `plotTranslation(data)`

This function generates a 2x2 grid of line plots, with each subplot representing the frame vs. translation (in meters) for a different object and various trackers. The plots are saved as `frame_vs_translation_all.png` in the `.\\benchmark\\results\\graphsAndValues\\` folder.

### `plotSpeedAny(data, obj='all', tracker='all')`

This function allows generating a custom frame vs. time plot for specific objects or trackers. By providing arguments `obj` and `tracker`, you can plot the frame vs. time for a particular object or tracker. The plots are saved with appropriate naming conventions.

### `plotRotationAny(data, obj='all', tracker='all')`

This function is similar to `plotSpeedAny`, but it plots the frame vs. rotation (in radians) for specific objects or trackers.

### `plotRotationZAny(data, obj='all', tracker='all')`

This function is similar to `plotSpeedAny` and `plotRotationAny`, but it plots the frame vs. rotation_z (another type of rotation) for specific objects or trackers.

### `plotTranslationAny(data, obj='all', tracker='all')`

This function is similar to the above functions, but it plots the frame vs. translation (in meters) for specific objects or trackers.
