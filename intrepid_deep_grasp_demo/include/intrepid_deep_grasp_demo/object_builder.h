#ifndef _COMPONENT_SORTING__OBJECT_BUILDER_H_
#define _COMPONENT_SORTING__OBJECT_BUILDER_H_

#include <ros/ros.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>
#include <iostream>
#include <string>

class Object_Builder
{
  public:
    
    Object_Builder(ros::NodeHandle pnh, std::string id);

    virtual ~Object_Builder();

    moveit_msgs::CollisionObject getObject();


  private:

    // Variables which hold config parameter yaml info

    ros::NodeHandle pnh_; // object paramenter node handle
    std::string id_; // Object id
    std::string frame_id_; // Frame used for object relative positioning
    XmlRpc::XmlRpcValue geometry_; // Stores object geometry parameter
    XmlRpc::XmlRpcValue pose_; // Stores object pose parameter
    moveit_msgs::CollisionObject collision_object_;
    
    // Variables created in Object class after processing parameter info 

    geometry_msgs::Pose pose_msg; // MoveIt Object Pose message
    shape_msgs::Mesh mesh; // Will be filled in only for mesh objects
    shape_msgs::SolidPrimitive primitive; // Will be filled in only for primitive shape objects
    
      
      
};

#endif // _COMPONENT_SORTING__OBJECT_BUILDER_H_