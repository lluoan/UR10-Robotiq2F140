# UR10-Robotiq2F140
## Desciption
This project aims to combine the Universal Robot and Robotiq together with ROS. The versions of robot arm and gripper are UR10 and Robotiq 2F-140 Grippers.
However, robotiq doesn't have an updated version ROS dependencies in ROS wiki which cause failure in building workspace. 
## Prerequisite
You need to download UR-Driver and UR-Description which are necessary for robot arm driving, motion planning and visualization.
The URL sources:

UR Driver: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

UR Decription: https://github.com/ros-industrial/universal_robot

You can also install them in ROS by:

```sudo apt-get install ros-noetic-ur-driver```

```sudo apt-get install ros-noetic-ur10-moveit-config```

Robotiq has a formal package provided in ROS wiki: http://wiki.ros.org/robotiq. But the highest distros is kinetic. When build this package in noetic workspace, the problems appear:

When I follow the tutorial of robotiq 2F gripper (http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29), I tried **rosdep install** the dependencies the package needed. But the distros versions of **python3-modbus** are imcompatitable. 

My solution is ignore this step and install a package which can provide modbus functions individually. You can download it from:

https://github.com/BoyangZhangFromHKUST/UR10-Robotiq2F140/tree/main/modbus

The control method of robotiq is from s-nam: https://github.com/s-nam/control_2f_gripper_and_ur10

## Control Strategy

