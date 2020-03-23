# kinect_tag_detection
A ROS package that can detect [AprilTags](https://april.eecs.umich.edu/software/apriltag) in streaming color images from Kinect v2 sensor.

## Table of contents
- [Requirements](#requirements)
- [Installation](#installation)
  - [Installation of iai_kinect2](#installation-of-iai_kinect2)
    - [Install libfreenect2](#install-libfreenect2)
    - [Test libfreenect2](#test-libfreenect2)
    - [Install iai_kinect2](#install-iai_kinect2)
    - [Test iai_kinect2](#test-iai_kinect2)
  - [Installation of apriltag_ros](#installation-of-apriltag_ros)
    - [Install apriltag_ros](#install-apriltag_ros)
    - [Test apriltag-ros](#test-apriltag_ros)
- [Usage](#usage)
- [References](#references)

## Requirements
* Kinect v2 sensor (e.g., Kinect for Windows)
* ROS machine (e.g., indigo) with USB 3.0 port
* Printed AprilTags (e.g., from [Tag Family](https://github.com/AprilRobotics/apriltag-imgs) 'tagStandard41h12')

## Installation
### Installation of iai_kinect2
#### Install libfreenect2
1. Follow the [steps](https://github.com/OpenKinect/libfreenect2#linux) until **build** step.
2. To **build**, run:
```
cd ~/libfreenect2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DENABLE_CXX11=ON
make
make install
```
3. Follow the left steps to complete the installation.
#### Test libfreenect2
Before the next installation please make sure **Protonect** is working and showing color, depth and ir images.
Connect Kinect v2 sensor and run:
```
cd ~/libfreenect2/build
./bin/Protonect cl                                                 # Test OpenCL support
./bin/Protonect gl                                                 # Test OpenGL support
./bin/Protonect cpu                                                # Test CPU support
```
#### Install iai_kinect2
1. Setup ROS environment:
```
export ROS_DISTRO=indigo                                           # Set this to your distro
source /opt/ros/$ROS_DISTRO/setup.bash                             # Source your ROS distro 
mkdir -p ~/catkin_ws/src                                           # Make a new catkin workspace if no one exists
cd ~/catkin_ws/src                                                 # Navigate to the source space
```
2. Follow the [steps](https://github.com/code-iai/iai_kinect2#install) until **catkin_make** step.
3. To **build**, instead of **catkin_make** run:
```
catkin build iai_kinect2
source ~/catkin_ws/devel/setup.bash
```
#### Test iai_kinect2
Before the next installation please make sure **[kinect2_bridge](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge)** is working (sensor calibration steps are unnecessary for this project).

0. Run the following three lines on every new shell you open or add them to your .bashrc:
   ```
   export ROS_DISTRO=indigo
   source /opt/ros/$ROS_DISTRO/setup.bash
   source ~/catkin_ws/devel/setup.bash
   ```
1. Connect Kinect v2 sensor, open a terminal for **kinect2_bridge**:
   ```
   roslaunch kinect2_bridge kinect2_bridge.launch
   ```
   If it runs properly, you should see prompt 'waiting for clients to connect'. 

2. Then, test **kinect2_bridge** in two ways:

* Use **[kinect2_viewer](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_viewer)**, open a second terminal for **kinect2_viewer**:
  ```
  rosrun kinect2_viewer kinect2_viewer sd image
  ```
  When streaming images are displayed, press Ctrl-C to terminate each terminal.

* Use **[rviz](http://wiki.ros.org/rviz/UserGuide)**.

  * Open a terminal and make sure that a roscore is up and running:
  ```
  roscore
  ```
  * If **kinect2_bridge** has not been running, do step 1.

  * Open a third terminal for **rviz**:
  ```
  rosrun rviz rviz
  ```
  Click 'Add' at the bottom left and create visualization by choosing one image topic shown in the list (an example is [here](https://github.com/Zhengro/kinect_tag_detection/blob/master/rviz_add_by_topic.png)). Press Ctrl-C to terminate each terminal.
### Installation of apriltag_ros
#### Install apriltag_ros
You can follow the [steps](https://github.com/AprilRobotics/apriltag_ros#quickstart) or directly run:
```
cd ~/catkin_ws/src                                                 # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git            # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git        # Clone Apriltag ROS wrapper
cd ~/catkin_ws                                                     # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y                 # Install any missing packages
catkin build apriltag_ros --cmake-args -DCMAKE_C_FLAGS="-std=c99"  # Build apriltag_ros
```
#### Test apriltag_ros

A tutorial of implementing detection in a video stream is available [here](http://wiki.ros.org/apriltag_ros/Tutorials/Detection%20in%20a%20video%20stream). In short, there are three steps:

0. Prepare AprilTags
   
   Scale up the [tags](https://github.com/AprilRobotics/apriltag-imgs/tree/master/tagStandard52h13) in your favorite editor and print them out. 

1. Parameter setup
  
   Always make sure the corresponding values match your actual tags.
   
   Three files requires modification. The working examples can be found:
   - [config/settings.yaml](https://github.com/Zhengro/kinect_tag_detection/blob/master/apriltag_ros/config/settings.yaml)
   - [config/tags.yaml](https://github.com/Zhengro/kinect_tag_detection/blob/master/apriltag_ros/config/tags.yaml)
   - [launch/continuous_detection.launch](https://github.com/Zhengro/kinect_tag_detection/blob/master/apriltag_ros/launch/continuous_detection.launch)
   
   Update the local files with them, run:
   ```
   cd ~
   wget https://raw.github.com/Zhengro/kinect_tag_detection/master/apriltag_ros/config/settings.yaml
   mv ~/settings.yaml ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/
   wget https://raw.github.com/Zhengro/kinect_tag_detection/master/apriltag_ros/config/tags.yaml
   mv ~/tags.yaml ~/catkin_ws/src/apriltag_ros/apriltag_ros/config/
   wget https://raw.github.com/Zhengro/kinect_tag_detection/master/apriltag_ros/launch/continuous_detection.launch
   mv ~/continuous_detection.launch ~/catkin_ws/src/apriltag_ros/apriltag_ros/launch/
   ```
2. Run the detector
   
   Enable image streaming and prepare **rviz**, i.e., go to step 2 (Use rviz) in [Test iai_kinect2](#test-iai_kinect2).

   Then open a terminal for **apriltag_ros**:
   ```
   roslaunch apriltag_ros continuous_detection.launch
   ```
   
   View the **/tag_detections_image** (image topic) in **rviz** like [here](https://github.com/Zhengro/kinect_tag_detection/blob/master/rviz_tag_detections_image.png).
   
   Press Ctrl-C to terminate each terminal.
   
## Usage

Follow the steps in [Test apriltag_ros](#test-apriltag_ros) to run the detector. Then, download some scripts for different functions.

1. To check camera info, run:
```
cd ~/catkin_ws/src/iai_kinect2/kinect2_bridge/
mkdir scripts & cd scripts
wget https://raw.github.com/Zhengro/kinect_tag_detection/master/kinect2_bridge/scripts/kinect2_caminfo_subscriber.py
```
```
rosrun kinect2_bridge kinect2_caminfo_subscriber.py
```
2. To check received color image with opencv, run:
```
cd ~/catkin_ws/src/iai_kinect2/kinect2_bridge/
mkdir scripts & cd scripts
wget https://raw.github.com/Zhengro/kinect_tag_detection/master/kinect2_bridge/scripts/kinect2_image_subscriber.py
```
```
rosrun kinect2_bridge kinect2_image_subscriber.py
```
3. To extract detected tag ids and then republish them to a new topic, run:
```
cd ~/catkin_ws/src/apriltag_ros/apriltag_ros/scripts/
wget https://raw.github.com/Zhengro/kinect_tag_detection/master/apriltag_ros/scripts/tag_ids_publisher.py
```
```
rosrun apriltag_ros tag_ids_publisher.py
```
4. (TO DO: To extract detected tag poses and then republish them to a new topic)

## References

* [libfreenect2: Release 0.2](https://github.com/OpenKinect/libfreenect2)
* [IAI Kinect2](https://github.com/code-iai/iai_kinect2)
* [RViz](https://github.com/ros-visualization/rviz)
* [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)
