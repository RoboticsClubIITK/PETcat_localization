#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <stdlib.h>
#include "GaussianPF_PathPlanning.cpp"

using namespace std;

vector<float> lidar_data(360);
const float MAX_VEL = 2;
const float MAX_OMEGA = 0.2;

float get_steering_msg(vector<float> lidar_dat, float goal_angle){
    // std::reverse(lidar_dat.begin(), lidar_dat.end());
    vector<float> ranges;
    ranges.insert(ranges.end(), lidar_dat.begin()+180,lidar_dat.end());
    ranges.insert(ranges.end(), lidar_dat.begin(), lidar_dat.begin()+180);
    assert(ranges.size() ==360);
    
    float h_theta = get_header_rad(ranges, goal_angle, false);
    cout << h_theta << endl;
    return -1*mapx(0, h_theta, MAX_OMEGA);
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan){ // Returns the lidar data
    lidar_data = scan->ranges;
    assert(lidar_data.size() == 360); // Ensuring that the lidar data has been successfully recvd. Replace 360 with 360/(Angular least count for lidar)
    cout << "Lidar data recvd:\n";
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lidar_subscriber");
    ros::NodeHandle sub_node;

    ros::Subscriber lid_sub = sub_node.subscribe("/husky/laser/scan", 1000, lidarCallback);

    ros::NodeHandle pub_node;

    ros::Publisher lid_pub = pub_node.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 100);

    ros::Rate rate(10); // sets the loop to publish at a rate of 10Hz
    float goal_angle = 0;
    while(ros::ok()){
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5;
        msg.angular.z = get_steering_msg(lidar_data, goal_angle);
        lid_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}