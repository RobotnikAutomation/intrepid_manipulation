# Intrepid manipulation 
Repository created to develop and try different manipulation applications using MoveIt! with the KINOVA JACO j2s6s200 (Jaco with two fingers).

# How to use MoveIt! with simulated robot

To launch robot gazebo simulation: 

`roslaunch kinova_gazebo robot_launch_j2s6s200.launch`

* ros_control is used to control the robot model in Gazebo (A Trajectory controller is used to provide interface for MoveIt!)

To launch MoveIt's move_group node + RViz interface: 

`roslaunch j2s6s200_moveit_config j2s6s200_gazebo_demo.launch`

# How to use MoveIt! with real robot

Previous configuration: 
 - Arm should be connected via ethernet to computer. 

To launch the essential drivers and configurations for the kinova arm (interface between hardware and ROS):

`roslaunch kinova_bringup kinova_robot_j2s6s200.launch`

To launch MoveIt's move_group node + RViz interface: 

`roslaunch j2s6s200_moveit_config j2s6s200_demo.launch`

* This also launches the joint_trajectory_action_server and gripper_command_action_server which are interfaces between MoveIt's commands and the robot's driver. 

