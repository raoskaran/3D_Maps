# 3D Maps

3D map generation using meshing algorithms on XYZ pointclouds created from Intel RealSense Depth and Tracking cameras.

## Installation instructions

#### 1. Install the librealsense module

Refer to https://github.com/IntelRealSense/librealsense for instructions on installing the librealsense module.

For OSX one may use :

	brew install librealsense

The viewer and other files will be stored in /usr/local/Cellar/librealsense/x.xx.x/bin

#### 2. Install the pyrealsense2 wrapper

For standard instructions refer to https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python

There is a shell script provided in the scripts directory for installation on Mac OS. Edit the script to add your own paths. After running the script, to check installation:

	>>>import sys
	>>>sys.path.append('/usr/local/lib') # Forgetting to do this will result in an import error for pyrealsense2
	>>>import pyrealsense2 as rs

Or if you want to avoid appending the path, either use export PYTHONPATH=$PYTHONPATH:/usr/local/lib or add PYTHONPATH=$PYTHONPATH:/usr/local/lib to .bash_profile and .bashrc files.

#### 3. Clone the repository
	
	git clone --recurse-submodules https://github.com/raoskaran/3D_Maps.git
	
	
#### 4. Install the requirements of the project

	pip install -r requirements.txt
	
## Pointcloud Generation (Refer to pcd_generation folder)

#### 1. Single frame generation
	
	This code is used for generating a point cloud from a single frame.
	
	Steps:
	
	a. Point the depth camera towards the object/scene you want to capture
	b. Run single_frame_gen.py
	c. The single frame will be plotted in matplotlib
	d. The output is stored as a txt file
	
#### 2. Discrete frame generation
	
	This code is used for generating multiple point clouds from multiple frames.
	
	Steps:
	
	a. Point the depth camera towards the object/scene you want to capture
	b. Run discrete_gen.py
	c. After one frame is captured, the code reiterates to capture the next frame where the camera is pointing
	d. Before moving on to the next frame, the relevant region of interest is calculated
	e. After capturing the required number of frames, press ctrl+C to exit
	f. The scene is plotted in matplotlib
	g. Each frame is stored in a separate txt file
	
#### 3. Continuous frame generation
	
	This code is used for generating multiple point clouds from multiple frames.
	
	Steps:
	
	a. Point the depth camera towards the object/scene you want to capture
	b. Run continuous_gen.py
	c. After one frame is captured, the code reiterates to capture the next frame where the camera is pointing
	d. Before moving on to the next frame, the relevant region of interest is calculated
	e. After capturing the required number of frames, press ctrl+C to exit
	f. The scene is plotted in matplotlib
	g. The combined scene is stored in a single txt file
	
## Scene Generation 

To create .xyz files from the .txt pointclouds, use xyzmaker.py and provide the path to the folder containing txt files as a command line argument. For algorithms other than delaunay triangulation, refer to visualization_techniques.py

	Eg: python3 xyzmaker.py path/to/directory
	
#### 1. Single frame map generation
	
	This code is used for generating a map out of singularly captured frames i.e. outputs from single frame generation. Eg: One frame contains a wall, the other contains a corner and so on.
	
	Steps:
	
	a. Change the path in singleframe_mapping.py to include the directory containing the separately captured .xyz files
	b. Run singleframe_mapping.py
	c. The code runs delaunay triangulation on each frame and combines them together to form a complete scene
	d. Pyvista plots the scene which can be visualized
	e. The scene is saved as a .stl file

#### 2. Discrete map generation
	
	This code is used for generating a map out of multiple captured frames i.e. outputs from discrete generation.
	
	Steps:
	
	a. Change the path in discrete_mapping.py to include the directory containing the captured .xyz files
	b. Run discrete_mapping.py
	c. The code runs delaunay triangulation on each frame and combines them together to form a complete scene
	d. Pyvista plots the scene which can be visualized
	e. The scene is saved as a .stl file
	
#### 3. Continuous map generation
	
	This code is used for generating a map out of multiple frames in a single file i.e. outputs from continuous generation.
	
	Steps:
	
	a. Change the path in continuous_mapping.py to include the directory containing the scene .xyz file
	b. Run continuous_mapping.py
	c. The code runs delaunay triangulation on the entire pointset to form a complete scene
	d. Pyvista plots the scene which can be visualized
	e. The scene is saved as a .stl file

## Hardware used

1. Intel® RealSense™ Depth Camera D435
2. Intel® RealSense™ Tracking Camera T265

## Software used

1. Python3
2. Open3D
3. PyVista
4. LibRealSense SDK
5. Pyrealsense library
	
## Citations

**Open3D**

@article{Zhou2018,
	author    = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
	title     = {{Open3D}: {A} Modern Library for {3D} Data Processing},
	journal   = {arXiv:1801.09847},
	year      = {2018},
}
