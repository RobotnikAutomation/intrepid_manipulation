#include <intrepid_deep_grasp_demo/pose_builder.h>

Pose_Builder::Pose_Builder(ros::NodeHandle pnh)
{
    // Get parameters from parameter server

    pnh_ = pnh;
    pnh_.getParam("frame_id", frame_id_); 
    pnh_.getParam("pose", pose_); 

    pose_stamped_msg.header.frame_id = frame_id_;

    // Process parameter pose

    if (pose_.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_.size() == 2)
    {    
        XmlRpc::XmlRpcValue pose_position;
        XmlRpc::XmlRpcValue pose_orientation;
        tf2::Quaternion quaternion_orientation;

        pose_position = pose_[0];
        pose_orientation = pose_[1];

        if (pose_position.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_position.size() == 3){
           
            pose_stamped_msg.pose.position.x = (pose_position[0]);
            pose_stamped_msg.pose.position.y = (pose_position[1]);
            pose_stamped_msg.pose.position.z = (pose_position[2]); 

        }else{
            ROS_ERROR("Cannot process pose position parameter, it should contain [x,y,z] array, check object configuration yaml");
        }

        if (pose_orientation.getType() == XmlRpc::XmlRpcValue::TypeArray && pose_orientation.size() == 3){

            quaternion_orientation.setRPY( pose_orientation[0], pose_orientation[1], pose_orientation[2]);

            pose_stamped_msg.pose.orientation.x = quaternion_orientation[0];
            pose_stamped_msg.pose.orientation.y = quaternion_orientation[1];
            pose_stamped_msg.pose.orientation.z = quaternion_orientation[2];     
            pose_stamped_msg.pose.orientation.w = quaternion_orientation[3];             
        }else{
            ROS_ERROR("Cannot process pose orientation parameter, it should contain [r,p,y] array, check object configuration yaml");
        }

    }else{
        ROS_ERROR("Cannot process pose parameter, it should contain [[x,y,z],[r,p,y]] list, check object configuration yaml");
    }
}

Pose_Builder::~Pose_Builder(){
    
};


geometry_msgs::PoseStamped Pose_Builder::getPose(){
    return pose_stamped_msg;
}

void Pose_Builder::setInit(){
    is_init = true;
}

bool Pose_Builder::isInit(){
    return is_init;
}