#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "pedsim_msgs/AgentStates.h"

#include <vector>
#include <string>
// TF
#include <tf/transform_listener.h>

// Custom utils
#include "localmap_utils.hpp"
class PathAssessment
{
public:
    // ROS related
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber model_states_sub_;
    ros::Publisher pub_friendlymap_;
    PathAssessment(ros::NodeHandle& nh);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    nav_msgs::OccupancyGrid::Ptr friendlymap_ptr;
    std::string localmap_frameid_;   
    // TF listener
    tf::TransformListener* tflistener_ptr_;

    //param
    int width,height;
    float resolution;

    //flag
    bool map_set =false; 
    bool map_init =false; 
    

};
PathAssessment::PathAssessment(ros::NodeHandle& nh) : nh_(nh)
{
    // ROS parameters
    ros::param::param<std::string>("~localmap_frameid", localmap_frameid_, "map");
    
    // ROS publishers & subscribers
    map_sub_ = nh_.subscribe("/map", 1, &PathAssessment::mapCallback, this);
    model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &PathAssessment::modelStatesCallback, this);
    pub_friendlymap_ = nh_.advertise<nav_msgs::OccupancyGrid>("human_friendly_map", 1);
}
void PathAssessment::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    // 处理地图信息
    // 例如，您可以输出地图的分辨率、宽度、高度等信息
    // ROS_INFO("Received map: resolution=%.2f, width=%d, height=%d", map_msg->info.resolution, map_msg->info.width, map_msg->info.height);
    width =  map_msg->info.width;
    height = map_msg->info.height;
    resolution = map_msg->info.resolution;
    map_set = true;
}
void PathAssessment::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    std::vector<std::string> model_names = msg->name;
    std::vector<geometry_msgs::Pose> poses = msg->pose;
    std::vector<geometry_msgs::Twist> twists = msg->twist;
    // 找到名为 "mars_lite" 的模型
    int mars_lite_index = -1;
    std::vector<int> actor_indices;
    for (size_t i = 0; i < model_names.size(); ++i)
    {
        if (model_names[i] == "mars_lite")
        {
            mars_lite_index = i;
        }
        else if (model_names[i].find("actor_") != std::string::npos && model_names[i].find("_adult") != std::string::npos)
        {
            // 如果模型名包含 "actor_" 和 "_adult"，则是人类模型
            actor_indices.push_back(i);
        }
    }
    // 输出 "mars_lite" 模型的姿态信息
    if (mars_lite_index != -1)
    {
        geometry_msgs::Pose mars_lite_pose = poses[mars_lite_index];
        geometry_msgs::Twist mars_lite_twist = twists[mars_lite_index];
        ROS_INFO("Pose of mars_lite: [x: %f, y: %f, z: %f] v:[%.2f, %.2f, %.2f]", mars_lite_pose.position.x, mars_lite_pose.position.y, mars_lite_pose.position.z, mars_lite_twist.linear.x, mars_lite_twist.linear.y, mars_lite_twist.linear.z);
    }
    else
    {
        ROS_INFO("Model 'mars_lite' not found in the received message.");
    }
    // 输出人类模型的姿态信息
    // for (size_t j = 0; j < actor_indices.size(); ++j)
    // {
        // geometry_msgs::Pose actor_pose = poses[actor_indices[0]];
        // geometry_msgs::Twist actor_twist = twists[actor_indices[0]];
        // ROS_INFO("Pose of actor %d: p: [%.2f, %.2f, %.2f] v:[%.2f, %.2f, %.2f]", 0, actor_pose.position.x, actor_pose.position.y, actor_pose.position.z
        // , actor_twist.linear.x, actor_twist.linear.y, actor_twist.linear.z);
    // }
    

    // Initialize human-friendly map meta information
    if(map_set){
        if(!map_init){
            friendlymap_ptr = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
            friendlymap_ptr->info.width = width;      // map_width --> x axis
            friendlymap_ptr->info.height = height;     // map_height --> y_axis
            friendlymap_ptr->info.resolution = resolution;
            friendlymap_ptr->info.origin.position.x = -resolution*width / 2;
            friendlymap_ptr->info.origin.position.y = -resolution*height / 2;
            friendlymap_ptr->info.origin.orientation.w = 1.0;
            friendlymap_ptr->data.resize((width/resolution) * (height/resolution));
            friendlymap_ptr->header.frame_id = localmap_frameid_;           
            map_init = true; 
        }        
    }
    //generate human-friendly map
    if(map_init){
        //todo

        // Localmap init
        std::fill(friendlymap_ptr->data.begin(), friendlymap_ptr->data.end(), 0);

        double map_origin_x = friendlymap_ptr->info.origin.position.x;
        double map_origin_y = friendlymap_ptr->info.origin.position.y;
        int map_limit = width * height - 1;  

        for(int j = 0; j < actor_indices.size(); j++) {
            // Convert object pose from laser coordinate to base coordinate
            double yaw, pitch, roll;
            // tf::Vector3 pt_laser(poses[actor_indices[j]].position.x, poses[actor_indices[j]].position.y, 0);
            // tf::Vector3 pt_base = tf_trk2base * pt_laser;
            tf::Quaternion q ;
            tf::quaternionMsgToTF(poses[actor_indices[j]].orientation, q);
            tf::Matrix3x3 mat(q);
            mat.getEulerYPR(yaw, pitch, roll);

            double speed = std::hypot(twists[actor_indices[j]].linear.x, twists[actor_indices[j]].linear.y);

            // Calculate object position in local map
            int map_x = (poses[actor_indices[j]].position.x - map_origin_x) / resolution;
            int map_y = (poses[actor_indices[j]].position.y - map_origin_y) / resolution;
            int idx = map_y * width + map_x;

            // Apply AGF
            localmap_utils::apply_social_agf(friendlymap_ptr, idx, yaw, speed, 100, true);

        }        



        // Publish localmap
        ros::Time now = ros::Time(0);
        friendlymap_ptr->header.stamp = now;
        pub_friendlymap_.publish(*friendlymap_ptr);
    }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_assessment");
    ros::NodeHandle nh;

    PathAssessment path_assessment(nh);

    ros::spin();

    return 0;
}