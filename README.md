# UR10-Robotiq2F140
## Desciption
This project aims to combine the Universal Robot and Robotiq together with ROS. The versions of robot arm and gripper are UR10 and Robotiq 2F-140 Grippers.
However, robotiq doesn't have an updated version ROS dependencies in ROS wiki which cause failure in building workspace. Therefore, we reference the method from others.
## Prerequisite
You need to download UR-Driver and UR-Description which are necessary for robot arm driving, motion planning and visualization.
The URL sources:

UR Driver: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

UR Decription: https://github.com/ros-industrial/universal_robot

You can also install them in ROS by:

```sudo apt-get install ros-noetic-ur-driver```

```sudo apt-get install ros-noetic-ur10-moveit-config```

Robotiq has a formal package provided in ROS wiki: http://wiki.ros.org/robotiq. But the highest distros is kinetic. When build this package in noetic workspace, the probelms appear:
