#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

namespace simulator {

class PoseController {
    public:
        PoseController(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        
    private:
        void poseCallback(const geometry_msgs::PoseStamped& msg);
        ros::Subscriber pose_sub_;
        ros::Publisher traj_pub_;
        std::vector<std::string> joint_name_list_;
        std::string quad_frame_id_;
        tf::TransformListener tf_listener_;
};

} // namespace simulator
