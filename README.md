# 3D_laser_scan
This node performs a 3D reconstruction using a 2D laser mounted horizontally and a Pan-Tilt servo Unit(PTU).
The PTU is controlled by a Arbotix board and the servos used are AX-18. The laser used is a 30m Hokuyo laser. 

This package includes the laser driver (URG-NODE), arbotix driver (Arbotix ROS), laser filter, and a own node called 3D_laser_scan. The third-party lybraries links are included at the end of this file. 

The 3D_laser_scan node package has a tower controller (tower_teleop.py) which uses a keyboard as an input to control some functionalities of the PTU.
The main inputs are "0" = set pan & tilt servos to 0 position. "S" start a 360 rotation of the horizontal servo. 

This package also includes a 3D_laser_scan.cpp which it gets the servo orientation and the 2D laser scan and creates a 3D point cloud. This node has some services such as save_pc or clear_pc. 

Arbotix drivers can be downloaded: https://github.com/vanadiumlabs/arbotix_ros

URG-node has been installed using  sudo apt-get install ros-kinetic-urg-node

Laser filter wiki: http://wiki.ros.org/laser_filters 
Laser filter node can be downloaded https://github.com/ros-perception/laser_filters
