# xArmRosPlayground

Description: At the moment this is implementing a school project for picking up boxes and putting them in containers.
This is the starting point of the project from which it could grow up to be something more formal, scalable and innovative.


### Install:
This project was originally built for ROS Melodic but we are transitioning to ROS Noetic. So, as a general dependency is usefull to install
`ros-noetic-desktop-full`

### Setup:
```
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### Gazebo:
This is the physics simulator for the environment. Here we include the physics of the boxes, the containers, and the table.

### Moveit:
We use moveit to control the xArm6, this includes path planning, trajectory definitions, control, and even vision (not added yet).
For further reference, please visit: https://moveit.ros.org/ 


### How to run the project:

Run gazebo simulation with: 
```
roslaunch xarm_gazebo xarm6_challenge.launch
```

Moveit control:
```
roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch
```

TasksPlanner:
```
rosrun path_planner TasksPlanner.py 2>&1 | ./ignore_warn.py
```

Solution (main program that operates the xArm):
```
rosrun pick_place solution_template.py 2>&1 | ./ignore_warn.py
```

### Todo's:
- Control xarm joint velocities
- Setup moveit collision objects since beginning
- Suppress TF_REPEATED_DATA warnings (this is the reason ignore_warn.py exists, as a temporal workaround to this)
- Build a node to sync Gazebo environment with Moveit environment


### Special thanks:

Special thanks to my fellow engineer Ricardo Leija for implementing the main setup and codebase for solution_template.py, Mauro Vaquero
and Maria Fernanda for helping out with the documentation for the submission of this assignature. And special thanks to my girlfriend
[Cristina Castillo](https://github.com/CrisCastilloM) for sticking with me along this journey, brainstorming,
helping me bounce my ideas and debuging with me <3.
