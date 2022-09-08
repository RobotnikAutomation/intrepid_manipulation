#include <ros/ros.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// CPP
#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <vector>

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

/* #include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h> */

// CUSTOM PLANNING SCENE BUIDLING CLASSES
#include <intrepid_deep_grasp_demo/object_builder.h>
#include <intrepid_deep_grasp_demo/pose_builder.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


constexpr char LOGNAME[] = "calculate_transform";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "calculate_transform", ros::init_options::AnonymousName);

  geometry_msgs::Pose link_5_to_cam_op, cam_base_to_cam_op;
  tf::Transform link_5_to_cam_op_tf, cam_base_to_cam_op_tf;

  link_5_to_cam_op.position.x = 0.043414;
  link_5_to_cam_op.position.y = 0.0752166;
  link_5_to_cam_op.position.z = 0.0315496;
  link_5_to_cam_op.orientation.x = -0.492943;
  link_5_to_cam_op.orientation.y = 0.517356;
  link_5_to_cam_op.orientation.z = 0.500177;
  link_5_to_cam_op.orientation.w = 0.489052;

  cam_base_to_cam_op.position.x = -0.001;
  cam_base_to_cam_op.position.y = 0.015;
  cam_base_to_cam_op.position.z = -0.000;
  cam_base_to_cam_op.orientation.x = -0.499;
  cam_base_to_cam_op.orientation.y = 0.500;
  cam_base_to_cam_op.orientation.z = -0.499;
  cam_base_to_cam_op.orientation.w = 0.501;

  tf::poseMsgToTF(link_5_to_cam_op,link_5_to_cam_op_tf);
  tf::poseMsgToTF(cam_base_to_cam_op,cam_base_to_cam_op_tf);

  tf::Transform result_tf = link_5_to_cam_op_tf * cam_base_to_cam_op_tf.inverse();
  geometry_msgs::Pose result;

  tf::poseTFToMsg(result_tf, result);

  ROS_INFO_STREAM (" TF: " << result);



 
  return 0;
}
