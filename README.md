# yoloSAnet_package: Ros Package for real-time object detection and state estimation using Yolo and RCNN-LSTM architecture

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [Motivation](#about-the-project)
* [Libraries Used](#prerequisites)
* [Files](#files)
* [Summary of Results](#summary)
* [Contact](#contact)
* [Acknowledgements](#acknowledgements)

## Motivation
The goal of this project is to use [Yolo-version5](https://github.com/s-arora-1987/yolov5) real-time for detecting objects (onions specifically) being sorted by a human trainer. After real-time detection of objects, the [SA-Net](http://thinc.cs.uga.edu/files/sahdICRA20.pdf) developed in thinc Lab is used to estimate the MDP states of domain defined in [PaperME-MTIRL](https://arxiv.org/abs/2004.12873). 

## Libraries Used 
We use the packages listed in tf3_requirements.txt for running Yolo. For camera setup, we ros package [iai_kinect2](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge) for kinect version 2 

### Setting up ROS environment for Python 3.* without conda
For running yolo package node: 
- tf3_requirements.txt gives the list of packages needed by yolol+sanet. without making a conda environment, install most (if not all packages).  
- add path to python3 packages 
export PYTHONPATH=<your-home>/catkin_ws/devel/lib/python3/dist-packages:/usr/lib/python3/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages

### Setting up and calibrating Camera
Read the instructions in calibration section of iai_kinect2 repository. Understand how serial number is given to camera and calibration files are stored in iai_kinect2/kinect_bridge/data folder. Make sure that calibration files in kinect2_calibration_results/calib_*.yaml are at the location specified. 

##### Note: running camera node kinect2_bridge as a nodelet gives bod-timeout related issues, which makes kinect_bridge nodelet version crashes a lot. So use: roslaunch kinect2_bridge kinect2_bridge.launch fps_limit:=5 use_nodelet:=false

## Files
[Yolo-version5](https://github.com/s-arora-1987/yolov5) is a sub-module of this ros package repository. The file detect_updated.py in [Yolo-version5](https://github.com/s-arora-1987/yolov5) has the code that receives images from camera node, passes them thorugh Yolo, and publishes output on a ros topic. 

For running object recognition, run camera node. Then use following command in separate terminal: 
rosrun yoloSAnet_package detect_updated.py

(optional) checking camera view during execution:
rosrun image_view image_view image:=/kinect2/hd/image_color

(optional) You can verify the detection the folder scripts/yolo/inference/output for checking saved images with bounding boxes.

## Summary of Results
The detected objects and states are shown in gifs in the main repository [IRL_projects](https://github.com/s-arora-1987/sawyer_i2rl_project_workspace) for this project.

### Next step
Intergrate the SA-Net PyTroch version in sciprts/sanet.ipynb in this ros package. 

## Contact
email: sa08751@uga.edu

## Acknowledgements
- [lab-mates](http://thinc.cs.uga.edu/#person) 
