#ifndef _DEEP_GRASP_DEMO__DEEP_GRASP_DEMO_H_
#define _DEEP_GRASP_DEMO__DEEP_GRASP_DEMO_H_

#include <ros/ros.h>

#include <rcomponent/rcomponent.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/server/simple_action_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
/* #include <component_sorting_msgs/PickupFromAction.h>
#include <component_sorting_msgs/PlaceOnAction.h>
#include <component_sorting_msgs/InitHolderAction.h> */

#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <vector>

/* #include <ur_msgs/SetIO.h>

#include <gazebo_ros_link_attacher/Attach.h> */

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <intrepid_deep_grasp_demo/object_builder.h>
#include <intrepid_deep_grasp_demo/pose_builder.h>

/* class ComponentSorting : public rcomponent::RComponent
{
public:
  ComponentSorting(ros::NodeHandle h);
  ~ComponentSorting();

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
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

  // MoveIt stuff
  void pick_chain_movement(std::string pick_position);
  void place_chain_movement(std::string place_position);
  void scan(std::string scanning_position);
  bool create_planning_scene();
  void gripper_on();
  void gripper_off();

  std::string group_name_;
  ros::WallDuration move_group_timeout_;
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  collision_detection::AllowedCollisionMatrix acm_;
  moveit_msgs::PlanningScene planning_scene_msg;


  std::vector< std::string > objects_in_roi; //CHANGE
  
  std::vector<Object_Builder> parsed_objects;
  std::vector<moveit_msgs::CollisionObject> moveit_objects;

  std::string action_;
  //actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>::GoalConstPtr pickup_from_goal_;
  //actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>::GoalConstPtr place_on_goal_;
  //actionlib::SimpleActionServer<component_sorting_msgs::InitHolderAction>::GoalConstPtr init_holder_goal_;
  //std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PickupFromAction>> pickup_from_as_;
  //std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::PlaceOnAction>> place_on_as_;
  //std::shared_ptr<actionlib::SimpleActionServer<component_sorting_msgs::InitHolderAction>> init_holder_as_;

  //component_sorting_msgs::PlaceOnFeedback place_feedback_;
  //component_sorting_msgs::PlaceOnResult place_result_;
  //component_sorting_msgs::PickupFromFeedback pick_feedback_;
  //component_sorting_msgs::PickupFromResult pick_result_;
  //component_sorting_msgs::InitHolderFeedback init_holder_feedback_;
  //component_sorting_msgs::InitHolderResult init_holder_result_;

  warehouse_ros::DatabaseConnection::Ptr conn_;

  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose current_cartesian_pose;
  geometry_msgs::Pose waypoint_cartesian_pose;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double success_cartesian_plan;
  double allowed_fraction_success = 0.95;
  bool success_plan;
  bool success_move;
  bool success_execute;

  const moveit::core::JointModelGroup* joint_model_group;
  robot_state::RobotStatePtr robot_state_;
 // moveit_visual_tools::MoveItVisualTools visual_tools_("robot_base_footprint");
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  
  std::string host;
  int port;
  float connection_timeout;
  int connection_retries;

  map<std::string, Pose_Builder> approach_poses_;
  map<std::string, Pose_Builder> place_poses_;
  map<std::string, Pose_Builder> pick_poses_;
  map<std::string, Pose_Builder> pre_pick_poses_;
  map<std::string, Pose_Builder> pre_place_poses_;

  std::vector<std::string> positions_to_use;

  //ur_msgs::SetIO srv;
  //gazebo_ros_link_attacher::Attach gazebo_link_attacher_msg;
  ros::ServiceClient gripper_client;
  ros::ServiceClient gazebo_link_attacher_client;
  ros::ServiceClient gazebo_link_detacher_client;

  //double dock_dist_table = 0.115;


  struct Box{
    double length, width;
  };

  struct Box box;

  double gripper_height;

  std::string identified_box;
  std::string identified_handle;

  double box_handle_displacement;

  bool simulation;

  std::string moveit_constraint;
  moveit_msgs::Constraints current_constraint;
  // in case we contact MoveIt through actionlib
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PickupAction>> pickup_as_;
  // std::shared_ptr<actionlib::SimpleActionServer<moveit_msgs::PlaceAction>> place_as_;

  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>> pickup_ac_;
  // std::shared_ptr<actionlib::SimpleActionClient<moveit_msgs::PlaceAction>> place_ac_;

  virtual void goalCB(const std::string& action);
  virtual void preemptCB();

  void tfLatchCallback();
  ros::Timer tf_latch_timer;

  std::vector <geometry_msgs::TransformStamped> latched_tf;
  tf2_ros::TransformBroadcaster tf_broadcaster;

  void tfListener(std::string scanning_position);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tf_listener;
  geometry_msgs::TransformStamped transform_stamped;
  geometry_msgs::TransformStamped table_qr_transform_stamped;
}; */

#endif  // _DEEP_GRASP_DEMO__DEEP_GRASP_DEMO_H_
