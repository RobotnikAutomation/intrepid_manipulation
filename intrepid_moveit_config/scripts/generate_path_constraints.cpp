#include <ros/ros.h>

#include <warehouse_ros/database_connection.h>
#include <moveit/warehouse/constraints_storage.h>

#include <moveit_msgs/Constraints.h>
constexpr char LOGNAME[] = "generate_path_constraints_database";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_path_constraints_database", ros::init_options::AnonymousName);

  std::string host = "localhost";
  int port = 33829;
  std::unique_ptr<moveit_warehouse::ConstraintsStorage> constraints_storage_;
  // Set up db
  try
  {
    warehouse_ros::DatabaseConnection::Ptr conn = moveit_warehouse::loadDatabase();
    conn->setParams(host, port);
    if (conn->connect())
    {
      constraints_storage_.reset(new moveit_warehouse::ConstraintsStorage(conn));
    }
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(LOGNAME, "%s", ex.what());
  }

  std::string robot = "intrepid";
  std::string group = "arm"; 
  std::vector<std::string> names;
  constraints_storage_->getKnownConstraints(names, robot, group);
  ROS_INFO_STREAM("Stored constraints " << names.size());
  for (auto n : names)
    ROS_INFO_STREAM(n);

  
    moveit_msgs::Constraints constraints;
    //constraints.header.frame_id = "robot_base_link";
    moveit_msgs::OrientationConstraint orientation;
    orientation.header.frame_id = "world";
    orientation.link_name = "j2s6s200_end_effector";
    orientation.orientation.x = 1;
    orientation.absolute_x_axis_tolerance = 0.1;
    orientation.absolute_y_axis_tolerance = 0.1;
    orientation.absolute_z_axis_tolerance = 3.15;
    orientation.weight = 100;
    constraints.name = "downright";
    constraints.orientation_constraints.clear();
    constraints.orientation_constraints.push_back(orientation);
    constraints_storage_->addConstraints(constraints, robot, group);

    constraints = moveit_msgs::Constraints();
    constraints.name = "shoulder_elbow";
    moveit_msgs::JointConstraint joint;
    constraints.joint_constraints.clear();
    joint.joint_name = "j2s6s200_joint_3";
    joint.position = 0.3316;
    joint.tolerance_below = 0.0;
    joint.tolerance_above = 2.4839;
    joint.weight = 100;
    constraints.joint_constraints.push_back(joint);
    joint.joint_name = "j2s6s200_joint_1";
    joint.position = 0;
    joint.tolerance_below = 3.1415; 
    joint.tolerance_above = 0.9893; 
    joint.weight = 100;
    constraints.joint_constraints.push_back(joint);
    constraints_storage_->addConstraints(constraints, robot, group);
    


  constraints_storage_->getKnownConstraints(names, robot, group);
  ROS_INFO_STREAM("Stored constraints after update " << names.size());
  for (auto n : names)
    ROS_INFO_STREAM(n);
  //    initializing_constraints_ = false;
  return 0;
}
