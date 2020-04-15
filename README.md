# 3D Maps

3D map generation using meshing algorithms on XYZ pointclouds created from Intel RealSense Depth and Tracking cameras.

## Installation instructions

#### 1. Install the librealsense module

	Refer to https://github.com/IntelRealSense/librealsense for instructions on installing the librealsense module.

	For OSX one may use :

	> brew install librealsense

	The viewer and other files will be stored in /usr/local/Cellar/librealsense/x.xx.x/bin

#### 2. Install the pyrealsense2 wrapper

	For standard instructions refer to https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python

	There is a shell script provided in the scripts directory for installation on Mac OS. Edit the script to add your own paths. After running the script, to check installation:
	
	>>>import sys
	>>>sys.path.append('/usr/local/lib') # Forgetting to do this will result in an import error for pyrealsense2
	>>>import pyrealsense2 as rs
	
	Or if you want to avoid appending the path, either use export PYTHONPATH=$PYTHONPATH:/usr/local/lib or add PYTHONPATH=$PYTHONPATH:/usr/local/lib to .bash_profile and .bashrc files.

#### 3. Install the requirements of the project

	> pip install -r requirements.txt
	
## Hardware used

	1. Intel® RealSense™ Depth Camera D435
	2. Intel® RealSense™ Tracking Camera T265
	
## Citations

**Open3D**

@article{Zhou2018,
	author    = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
	title     = {{Open3D}: {A} Modern Library for {3D} Data Processing},
	journal   = {arXiv:1801.09847},
	year      = {2018},
}
