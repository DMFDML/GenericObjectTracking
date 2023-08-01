# plotSingleRunData.py

The script reads a CSV file containing object tracking data for a specific tracker and generates a line plot to visualize the performance of the tracker over time.

## Script Overview

The script is designed to perform the following tasks:

1. Parse command-line arguments to specify the tracker and the CSV file containing benchmark data.
2. Read the benchmark data from the specified CSV file.
3. Calculate various metrics based on the data.
4. Generate a line plot to visualize the performance of the tracker over time.

## Script Usage

To use the script, follow these steps:

1. Run the script using Python with the appropriate command-line arguments.

```bash
python script_name.py --tracker TrackerName --file PathToCSVFile
```

Here, replace `TrackerName` with the name of the specific object tracker being analyzed, and `PathToCSVFile` with the path to the CSV file containing the benchmark data.