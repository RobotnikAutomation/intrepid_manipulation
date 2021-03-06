# Intrepid manipulation 
Intrepid manipulation application demo for Kinova Jaco j2s6s200, built using different frameworks:
 * MoveIt.
 * MoveIt Task Constructor.
 * Deep learning algorithms -> GPD (grasp pose detection) / Dexnet

# How to use application with simulated robot

To launch robot gazebo simulation: 

`roslaunch intrepid_simple_sim simulation.launch`

* ros_control is used to control the robot model in Gazebo (A Trajectory controller is used to provide interface for MoveIt!)

To launch MoveIt's move_group node + RViz interface: 

`roslaunch intrepid_moveit_config j2s6s200_gazebo_demo.launch`

To launch deep grasp manipulation application:

`roslaunch intrepid_deep_grasp_demo gpd_demo_gazebo.launch`

Use RViz's intrepid plugin to interact with application: 
 * Start Detection: Launches deep learning pick object action
 * Display Plan: Allows user to display moveit's planned motion path
 * Execute Trajectory: Allows user to execute moveit's planned path
 * Stop motion: Stop motion execution

# How to use application with real robot

Previous configuration: 
 - Arm should be connected via ethernet to computer. 

To launch the essential nodes: 

`roslaunch intrepid_bringup intrepid_manipulation_bringup.launch`

This launch file launches:

 * Essential hardware drivers and configurations for the kinova arm (interface between hardware and ROS).
 * 3D mouse driver.
 * Intel D435 driver.
 * Point cloud filtering node. 

To launch MoveIt's move_group node + RViz interface: 

`roslaunch j2s6s200_moveit_config j2s6s200_demo.launch`

* This also launches the joint_trajectory_action_server and gripper_command_action_server which are interfaces between MoveIt's commands and the robot's driver. 

To launch Moveit Servo node for teleoperation:

`roslaunch kinova_moveit_servo kinova_moveit_servo.launch`

To launch deep grasp manipulation application:

`roslaunch intrepid_deep_grasp_demo gpd_demo.launch`

Use RViz's intrepid plugin to interact with application: 
 * Start Detection: Launches deep learning pick object action
 * Display Plan: Allows user to display moveit's planned motion path
 * Execute Trajectory: Allows user to execute moveit's planned path
 * Stop motion: Stop motion execution
# Extra: Launch MoveIt Grasps Pipeline Demo

This demo launches a table and an object to grasp as moveit collision objects. To modify the size and position of the table and pick target (has to be a cube) use the objects_config.yaml, inside config_robot folder: 

`roslaunch intrepid_moveit_grasps_demo grasp_pipeline_demo.launch`


