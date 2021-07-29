#ifndef _INTREPID_DEEP_GRASP_DEMO__POSE_BUILDER_H_
#define _INTREPID_DEEP_GRASP_DEMO__POSE_BUILDER_H_

#include <ros/ros.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit_msgs/Constraints.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <iostream>
#include <string>

class Pose_Builder 
{
  public:
    
    Pose_Builder(ros::NodeHandle pnh);

    virtual ~Pose_Builder();

    geometry_msgs::PoseStamped getPose();

    void setInit();

    bool isInit();

  private:

    // Variables which hold config parameter yaml info

    ros::NodeHandle pnh_; // object paramenter node handle
    std::string frame_id_; // Frame used for object relative positioning
    XmlRpc::XmlRpcValue pose_; // Stores object pose parameter
    bool is_init = false;
    
    // Variables created in Object class after processing parameter info 

    geometry_msgs::PoseStamped pose_stamped_msg; // MoveIt Object Pose message
};

#endif // _INTREPID_DEEP_GRASP_DEMO__POSE_BUILDER_H_