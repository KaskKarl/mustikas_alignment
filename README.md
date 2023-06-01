# Mustikas alignment package
This is a ROS package which is used to align a xarm robotic arm with a blueberry plant using a intel realsense camera. DarkNet, DarkHelp and OpenCV are needed to build this package.

The package consists of several nodes. The nodes starting with nr 2 have multiple object handling capability.

The Gazebo simulation of the xarm and the Intel Realsense mounted to the robot frame can be launched with the file: `camera_on_robot_gazebo.launch`

Working on a real robot `camera_on_robot_real.launch` can be used. Argument `robot_ip` has to be set to connect to the xarm.

Different parameters can be changed within the launch files:
```
mustikas/camera/offset_x - camera's x coordinate in the robot coordinate system
mustikas/camera/offset_y - camera's y coordinate in the robot coordinate system
mustikas/camera/offset_z - camera's z coordinate in the robot coordinate system
mustikas/camera/angle - camera's mountingangle from the horizon
mustikas/xarm/angle - xarm's mountingangle from the horizon

mustikas/goal/offset_x - offset of the alignment position from the detected object in x axis
mustikas/goal/offset_y - offset of the alignment position from the detected object in y axis
mustikas/goal/offset_z - offset of the alignment position from the detected object in z axis

mustikas/fert_area/center_x - distance from the base of the robot to the center of the fertilizing area
mustikas/fert_area/center_y - fertilizing area offset from the center of the robot
mustikas/fert_area/width - width of the fertilizing area
mustikas/fert_area/length - length of the fertilising area
mustikas/igno_time - time when plants within the fertilization area are ignored after completing fertilization

mustikas/msg_time_limit - time after which alignment to the coordinates is not carried out
```

The process can be controlled by calling the `/xarm_control` service. There are 4 commands:
```
READY - the robot moves to the ready position
GOAL - the robot moves to the position provided by the commander_node and stays there
AUTO - the robot moves to the position provided by the commander_node and then back to the ready position. The cycle repeats as long as there are coordinates being published by commander_node
STOP - the movement of the robot is stopped
```
If the user wants to move the robot to the READY position, the command to be executed should be: `rosservice call /xarm_control "READY"`
