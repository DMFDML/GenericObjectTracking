# Introduction

This File contains all the code to run the 6D pose estimation algorithms

Docs:
* **benchmark**
  * [benchmark.py](docs\Benchmark\benchmark.md)
  * [plotData.py](/docs/Benchmark/plotData.md)
  * [plotSingleRunData.py](\docs\Benchmark\plotSingleRunData.md)
  * [saveBenchmarkImages.py](\docs\Benchmark\saveBenchmarkImages.md)
  * [showInUnity.py](\docs\Benchmark\showInUnity.md)
* [images](\docs\Images\images.md)
* **re3-tensorflow**
* **SiamMask**
* **src**
  * **Camera**
    * [AzureKinect.py](\docs\Camera\AzureKinect.md)
    * [Camera.py](\docs\Camera\Camera.md)
    * [FakeCamera.py](\docs\Camera\FakeCamera.md)
    * [OpenCVCamera.py](\docs\Camera\OpenCVCamera.md)
  * **ObjectTrackers**
    * [arucoTrack.py](\docs\ObjectTrackers\arucoTrack.md)
    * [genericObjectTracker.py](\docs\ObjectTrackers\genericObjectTracker.md)
    * [ObjectTrackerInterface.py](\docs\ObjectTrackers\ObjectTrackerInterface.md)
    * [reflectorTrack.py](\docs\ObjectTrackers\reflectorTrack.md)
    * [reflectorTrackNoICP.py](\docs\ObjectTrackers\reflectorTrackNoICP.md)
  * **Trackers2D**
    * [Re3.py](\docs\Trackers2D\Re3.md)
    * [SiamMask.py](\docs\Trackers2D\SiamMask.md)
    * [TrackerInterface.py](\docs\Trackers2D\TrackerInterface.md)
  * **Util**
    * [helperFunctions.py](\docs\Util\helperFunctions.md)
    * [objectTrackingConstants.py](\docs\Util\objectTrackingConstants.md)
    * [showPointCloud.py](\docs\Util\showPointCloud.md)
* **triad_openvr**


# Setup 

## SiamMask

Please clone the [SiamMask](https://github.com/foolwood/SiamMask#training-models) repository in this directory

Then type:
```
cd ./SiamMask
mkdir PreTrainedModels
wget http://www.robots.ox.ac.uk/~qwang/SiamMask_VOT.pth -UseBasicParsing -O SiamMask_VOT.pth
wget http://www.robots.ox.ac.uk/~qwang/SiamMask_DAVIS.pth -UseBasicParsing -O SiamMask_DAVIS.pth
```

## Re3

### Re3 tensorFlow

If you decide to use tensorflow you will need to download [Cuda toolkit 9.0](https://developer.nvidia.com/cuda-90-download-archive?target_os=Windows&target_arch=x86_64&target_version=10&target_type=exelocal) and [Download cuDNN v7.0.5 (Dec 5, 2017), for CUDA 9.0](https://developer.nvidia.com/rdp/cudnn-archive) 

Then add cuDNN to the Path:
- [Windows](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/)
- [Linux](https://linuxize.com/post/how-to-add-directory-to-path-in-linux/)

Please clone the [Re3](https://github.com/danielgordon10/re3-tensorflow) repository in this directory

### Re3 Pytorch

Please clone the [Re3](https://github.com/danielgordon10/re3-pytorch) repository in this directory

## Python

Please install [anaconda](https://www.anaconda.com/)

Use the command within this directory:
```
conda env create -f environment.yml
```

then run 
```
conda activate object_tracking
pip install torch===1.4.0  -f https://download.pytorch.org/whl/torch_stable.html
pip install torchvision===0.5.0
pip install tensorflow-gpu==1.9.0
pip install tensorflow-tensorboard==1.5.1
```

# Usage

To use a specific tracker:
1. Plug in the Azure kinect camera.
2. Make sure that the Camera being used in the code is AzureKinectCamera.
3. from this directory run python ./src/ObjectTrackers/genericObjectTracker.py.
4. When the video has loaded press `d` and draw a bounding box around the object wanting to be tracked.

