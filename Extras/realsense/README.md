ROS Wrapper for Intel® RealSense™ Devices
=================

These are packages for using Intel RealSense cameras with ROS.

LibRealSense2 supported version: v2.50.0 (see realsense2_camera release notes)


Installation Instructions
-------------------------

Ubuntu 20.04 and ROS Noetic

### Install realsense2_camera

realsense2_camera is available as a debian package of ROS distribution. It can be installed by typing:

     sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

This will install both realsense2_camera and its dependents, including librealsense2 library and matching udev-rules.

Notice:

* The version of librealsense2 is almost always behind the one availeable in RealSense™ official repository.
* ibrealsense2 is not built to use native v4l2 driver but the less stable RS-USB protocol. That is because the last is more general and operational on a larger variety of platforms.
* realsense2_description is available as a separate debian package of ROS distribution. It includes the 3D-models of the devices and is necessary for running launch files that include these models (i.e. rs_d435_camera_with_model.launch). It can be installed by typing:  
            
      sudo apt-get install ros-$ROS_DISTRO-realsense2-description    
 
### Start the camera node

To start the camera node in ROS:
	
	roslaunch realsense2_camera rs_camera.launch
	
### License
 
Copyright 2018 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with the License. You may obtain a copy of the License at
	
	http://www.apache.org/licenses/LICENSE-2.0
	
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others
