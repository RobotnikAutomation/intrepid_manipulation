#ifndef _INTREPID_DEEP_GRASP_DEMO__DEEP_GRASP_DEMO_H_
#define _INTREPID_DEEP_GRASP_DEMO__DEEP_GRASP_DEMO_H_

// CPP
#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <vector>
#include <boost/format.hpp>

// RVIZ

#include <jsk_rviz_plugins/OverlayText.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

//TF
//#include <tf/tf.h>
//#include <tf/transform_listener.h>


// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

//// Eigen
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

// MTC 
#include <moveit_task_constructor_msgs/SampleGraspPosesAction.h>

// MTC demo implementation
#include <intrepid_deep_grasp_demo/deep_pick_place_task.h>
#include <intrepid_deep_grasp_demo/deep_pick_task.h>
#include <intrepid_manipulation_msgs/PickupObjectAction.h>
#include <intrepid_manipulation_msgs/PickupFromAction.h>
#include <intrepid_manipulation_msgs/PlaceOnAction.h>
#include <actionlib_msgs/GoalID.h>


// RCOMPONENT
#include <rcomponent/rcomponent.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>

// CUSTOM PLANNING SCENE BUIDLING CLASSES
#include <intrepid_deep_grasp_demo/object_builder.h>
#include <intrepid_deep_grasp_demo/pose_builder.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class DeepGraspDemo : public rcomponent::RComponent
{
  typedef actionlib::SimpleActionClient<intrepid_manipulation_msgs::PickupObjectAction> DGDAC;

public:
  DeepGraspDemo(ros::NodeHandle h);
  ~DeepGraspDemo();

protected:
  // RComponent stuff

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads params from params server
  virtual void rosReadParams();

  virtual int setup();

  // States
  virtual void standbyState();
  virtual void readyState();
  

  // ROS stuff
  std::shared_ptr<tf2_ros::Buffer> move_group_tf2_buffer_; // Move_group buffer
  std::shared_ptr<tf2_ros::Buffer> move_group_gripper_tf2_buffer_; // Move_group buffer
  tf2_ros::Buffer tfBuffer; // TF buffer
  tf2_ros::TransformListener* tf_listener; // TF listener

  // MTC task
  std::shared_ptr<intrepid_deep_grasp_demo::DeepPickPlaceTask> deep_pick_place_task;
  std::shared_ptr<intrepid_deep_grasp_demo::DeepPickTask> deep_pick_task; // MTC deep grasp demo pick task object
  
  // Global Action server callbacks
  virtual void goalCB(const std::string& action);
  virtual void preemptCB();

  // Pick object Deep learning Action server
  std::shared_ptr<actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupObjectAction>> pick_object_as_;
  actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupObjectAction>::GoalConstPtr pick_object_goal_;
  intrepid_manipulation_msgs::PickupObjectFeedback pick_object_feedback_; 
  intrepid_manipulation_msgs::PickupObjectResult pick_object_result_;
  ros::Publisher pick_goal_pub; //  Action server goal topic publisher
  ros::Publisher pick_cancel_pub; // Action server cancel topic publisher

  // Pick and Place object Action servers
  std::shared_ptr<actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupFromAction>> pickup_from_as_;
  actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupFromAction>::GoalConstPtr pickup_from_goal_;
  intrepid_manipulation_msgs::PickupFromFeedback pickup_from_feedback_; 
  intrepid_manipulation_msgs::PickupFromResult pickup_from_result_;

  std::shared_ptr<actionlib::SimpleActionServer<intrepid_manipulation_msgs::PlaceOnAction>> place_on_as_;
  actionlib::SimpleActionServer<intrepid_manipulation_msgs::PlaceOnAction>::GoalConstPtr place_on_goal_;
  intrepid_manipulation_msgs::PlaceOnFeedback place_on_feedback_; 
  intrepid_manipulation_msgs::PlaceOnResult place_on_result_;

  // Pick object action client
  std::shared_ptr<DGDAC> pick_object_ac_;

  // Manipulation functions
  void pickup_from_pose(geometry_msgs::PoseStamped pose);
  void place_on_pose(geometry_msgs::PoseStamped pose);

  // RVIZ Intrepid GUI PLUGIN
  ros::Subscriber intrepid_gui_sub; // Intrepid RViz GUI Plugin subscriber 
  bool allow_execute_; // Allow execution of moveit motion plan
  bool allow_start_; // Allow start of deep learning action
  bool allow_display_; // Allow display of moveit plan
  bool stop_execution_; // Stop moveit motion execution 
  void intrepid_gui_callback(const sensor_msgs::Joy& msg); // Intrepid Rviz GUI Callback

  // RVIZ jsk text overlay display PLUGIN
  std_msgs::ColorRGBA set_text_color(std::vector<double>& rgba_color); // Function to set text color 
  std::vector<double> warning_color = {1, 0 ,0, 1};
  std::vector<double> success_color = {0, 1 ,0, 1};
  std::vector<double> feedback_color = {25/255.0, 1.0, 240.0/255.0, 1.0};
  std::vector<double> background_color = {0.0, 0.0 ,0.0, 0.0};
  ros::Publisher overlay_text_pub; // RViz overlay text publisher
  jsk_rviz_plugins::OverlayText rviz_text_msg; // Text msg to publish

  // ROS PARAMS FROM JACO DESCRIPTION YAML
  std::string arm_group_name_; // Move_group group name 
  std::string world_frame_; // Move_group world frame
  double look_dist_; // Distance in z over object 
  std::string host; // Mongo DB database host
  int port; // Mongo DB database port
  std::string point_cloud_frame_, end_effector_frame_; // Pointcloud camera frame and end effector frame
  std::string camera_in_pcl_topic_, merged_out_pcl_topic_; // PCL camera in topic and deep grasp demo out PCL topic

  // Moveit Initialization stuff
  ros::WallDuration move_group_timeout_; 
  moveit::planning_interface::MoveGroupInterfacePtr move_group_; // Move group interface
  moveit::planning_interface::MoveGroupInterfacePtr move_group_gripper_; // Move group interface
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_; // Planning scene interface
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; // Planning scene monitor
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  warehouse_ros::DatabaseConnection::Ptr conn_; // Moveit warehouse database connection
  bool create_planning_scene(); // Moveit planning scene upload function

  std::string desired_constraint_name_;
  moveit_msgs::Constraints global_constraint; 

  // Movegroup Motion stuff
  bool move_to_look_pose(geometry_msgs::PoseStamped pose); // Move to x,y,z position (without orientation) and rotate end-effector to put camera in place
  void move_rel_cartesian(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw, const moveit_msgs::Constraints& constraint); //Move end_effector relative to itself
  void move_to_cartesian(const geometry_msgs::Pose& cartesian_pose, const moveit_msgs::Constraints& constraint); // Move to a world cartesian position
  const double jump_threshold = 0.0; // Cartesian planner parameter
  const double eef_step = 0.005; // Cartesian planner parameter
  double allowed_fraction_success = 0.95;

  double success_cartesian_plan;
  bool success_plan;
  bool success_move;
  bool success_execute;
  
  // Deep learning pickup object functionalities
  bool scanObject(); // Move end effector to scan object
  double scan_move_linear_, scan_move_angular_; // Amount to move when scanning area
  geometry_msgs::Pose null_pose; // Define zero pose

  // Deep learning point cloud functionalities
  ros::Subscriber point_cloud_sub; // Subscriber to incoming point cloud (from camera or filtering node)
  sensor_msgs::PointCloud2 cloud_in_msg; // Stores incoming point cloud
  void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg); // Point cloud in callback
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud; // Global merged cloud point cloud object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_merged_cloud; // Global filtered merged cloud point cloud object
  void addPointCloud(); // Adds current point cloud to global merged cloud
  ros::Timer dgd_pcl_pub_timer; // Timer to publish filtered merged point cloud
  void pubPointCloudCallback(); // Timer callback
  ros::Publisher merged_cloud_pub_; // Publishes filtered merged cloud
  bool publish_cloud_; // Allows filtered merged cloud to be published

  // Publish computation time through RVIZ
  ros::Timer publish_computation_time_timer; // Timer thread to publish computation time in RVIZ GUI
  ros::WallTime timer_start_; // Sets start time 
  bool publish_computation_time; // Allows computation time to be published
  void publishComputationTimeCallback(); // Timer callback

  // CPP thread functionalities
  bool action_ready_flag_; // Flag to check status of action (if false, action is still executing)
};

#endif  // _INTREPID_DEEP_GRASP_DEMO__DEEP_GRASP_DEMO_H_
