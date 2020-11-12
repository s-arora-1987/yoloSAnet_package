# yoloSAnet_package

For running yolo package node: 
- tf3_requirements.txt gives the list of packages needed by yolol+sanet. without mking a conda environment, install most (if not all packages).  
- add path to python3 packages 
export PYTHONPATH=/home/psuresh/catkin_ws/devel/lib/python3/dist-packages:/usr/lib/python3/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages

Read the instructions in calibration section of iai_kinect2 repository. Understand how seial number is given to camera and calibration files are stored
iai_kinect2/kinect_bridge/data folder. Make sure that calibration files in kinect2_calibration_results/calib_*.yaml are at the location specified. 

For every terminal window, run
./intera.sh 
in catkin_ws

running camera. as nodelet gives bod-timeout related issues in intera shell, which makes kinect_bridge nodelet version crashes a lot. so use:
roslaunch kinect2_bridge kinect2_bridge.launch fps_limit:=5 use_nodelet:=false

running object recognition in python3:
rosrun yoloSAnet_package detect_updated.py

(optional) checking camera view during execution:
rosrun image_view image_view image:=/kinect2/hd/image_color

See the folder scripts/yolo/inference/output for checking saved images with bounding boxes
