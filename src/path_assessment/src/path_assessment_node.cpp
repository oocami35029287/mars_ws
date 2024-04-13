#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "pedsim_msgs/AgentStates.h"
#include "path_assessment/Assessment.h"
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
    ros::Subscriber agent_states_sub_;
    ros::Publisher pub_friendlymap_;
    ros::Publisher pub_assessment_;
    PathAssessment(ros::NodeHandle& nh);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void agentStatesCallback(const pedsim_msgs::AgentStates::ConstPtr& agent_states_msg);
    
    // TF listener
    tf::TransformListener* tflistener_ptr_;

    //param
    std::string localmap_frameid_;  
    int lower_h_threshold,upper_h_threshold;
    int lower_o_threshold,upper_o_threshold;
    double waiting_stop;

    nav_msgs::OccupancyGrid::Ptr friendlymap_ptr;
    nav_msgs::OccupancyGrid::ConstPtr obstaclemap_ptr;
    geometry_msgs::Pose robot_pose;
    geometry_msgs::Twist robot_twist;
    int width,height;
    float resolution;
    //flag
    bool map_set =false; 
    bool map_init =false; 

    //global val for evaluarion
    double eva_avg_h=1.0;
    double eva_avg_o=1.0;
    bool stop_count_flg = false;
    bool temp_stop_flg = false;
    bool run_flg = false;
    ros::Time stop_time;
    ros::Time start_time;
    int colli_h_cnt = 0;
    int colli_h = 0; //friendly collision
    bool colli_h_flg = false;
    double colli_h_time = 0;
    ros::Time colli_h_t_str;
    int colli_o_cnt = 0;
    int colli_o = 0; //friendly collision
    bool colli_o_flg = false;
    double colli_o_time = 0;
    ros::Time colli_o_t_str;
    bool output_date_flg = false;
    double total_time = 0;
};
PathAssessment::PathAssessment(ros::NodeHandle& nh) : nh_(nh)
{
    // ROS parameters
    ros::param::param<std::string>("~localmap_frameid", localmap_frameid_, "map");
    ros::param::param<int>("~lower_h_threshold", lower_h_threshold, 40); 
    ros::param::param<int>("~upper_h_threshold", upper_h_threshold, 100); 
    ros::param::param<int>("~lower_o_threshold", lower_o_threshold, 40); 
    ros::param::param<int>("~upper_o_threshold", upper_o_threshold, 100); 
    ros::param::param<double>("~waiting_stop", waiting_stop, 2.5); 
    // ROS publishers & subscribers
    map_sub_ = nh_.subscribe("/move_base/global_costmap/costmap", 1, &PathAssessment::mapCallback, this);
    model_states_sub_ = nh_.subscribe("/gazebo/model_states", 1, &PathAssessment::modelStatesCallback, this);
    agent_states_sub_ = nh_.subscribe("pedsim_simulator/simulated_agents", 1, &PathAssessment::agentStatesCallback, this);
    pub_friendlymap_ = nh_.advertise<nav_msgs::OccupancyGrid>("human_friendly_map", 1);
    pub_assessment_ = nh.advertise<path_assessment::Assessment>("path_assessment", 10);
}
void PathAssessment::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
    // 处理地图信息
    // 例如，您可以输出地图的分辨率、宽度、高度等信息
    obstaclemap_ptr = map_msg;  
    ROS_INFO("Received map: resolution=%.2f, width=%d, height=%d", map_msg->info.resolution, map_msg->info.width, map_msg->info.height);
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
        // else if (model_names[i].find("actor_") != std::string::npos && model_names[i].find("_adult") != std::string::npos)
        // {
        //     // 如果模型名包含 "actor_" 和 "_adult"，则是人类模型
        //     actor_indices.push_back(i);
        // }
    }
    // 输出 "mars_lite" 模型的姿态信息
    if (mars_lite_index != -1)
    {
        robot_pose = poses[mars_lite_index];
        robot_twist = twists[mars_lite_index];
        // ROS_INFO("Pose of mars_lite: [x: %f, y: %f, z: %f] v:[%.6f, %.6f, %.6f]", robot_pose.position.x, robot_pose.position.y, robot_pose.position.z, robot_twist.linear.x, robot_twist.linear.y, robot_twist.linear.z);
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

}
void PathAssessment::agentStatesCallback(const pedsim_msgs::AgentStates::ConstPtr& agent_states_msg)
{
    std::vector<geometry_msgs::Pose> poses;
    std::vector<geometry_msgs::Twist> twists;
    // Process agent states
    int agent_size = (agent_states_msg->agent_states).size();
    for (int i=0;i<agent_size;i++)
    {
        // Access each agent state
        int id = agent_states_msg->agent_states[i].id;
        geometry_msgs::Pose pose = agent_states_msg->agent_states[i].pose;
        geometry_msgs::Twist twist = agent_states_msg->agent_states[i].twist;
        poses.push_back(pose);
        twists.push_back(twist);
        // Process or use the pose and twist information as needed
        // ROS_INFO("Agent ID: %d, Pose [x: %f, y: %f, z: %f], Twist [linear_x: %f, linear_y: %f, angular_z: %f]", 
        //   id, pose.position.x, pose.position.y, pose.position.z, twist.linear.x, twist.linear.y, twist.angular.z);
    }

    // Initialize human-friendly map meta information
    if(map_set){
        if(!map_init){
            friendlymap_ptr = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
            friendlymap_ptr->info.width = width;      // map_width --> x axis
            friendlymap_ptr->info.height = height;     // map_height --> y_axis
            friendlymap_ptr->info.resolution = resolution;
            friendlymap_ptr->info.origin.position.x = obstaclemap_ptr->info.origin.position.x;
            friendlymap_ptr->info.origin.position.y = obstaclemap_ptr->info.origin.position.y;
            friendlymap_ptr->info.origin.orientation.w = obstaclemap_ptr->info.origin.orientation.w;
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

        for(int i = 0; i < agent_size; i++) {
            // Convert object pose from laser coordinate to base coordinate
            double yaw, pitch, roll;
            // tf::Vector3 pt_laser(poses[actor_indices[j]].position.x, poses[actor_indices[j]].position.y, 0);
            // tf::Vector3 pt_base = tf_trk2base * pt_laser;
            tf::Quaternion q ;
            tf::quaternionMsgToTF(poses[i].orientation, q);
            tf::Matrix3x3 mat(q);
            mat.getEulerYPR(yaw, pitch, roll);

            double speed = std::hypot(twists[i].linear.x, twists[i].linear.y);

            // Calculate object position in local map
            int agent_x = (poses[i].position.x - map_origin_x) / resolution;
            int agent_y = (poses[i].position.y - map_origin_y) / resolution;
            int agent_idx = agent_y * width + agent_x;

            // Apply AGF
            localmap_utils::apply_social_agf(friendlymap_ptr, agent_idx, yaw, speed, 100, true);

        }        



        // Publish localmap
        ros::Time now = ros::Time(0);
        friendlymap_ptr->header.stamp = now;
        pub_friendlymap_.publish(*friendlymap_ptr);
        
        int robot_x = (robot_pose.position.x - map_origin_x) / resolution;
        int robot_y = (robot_pose.position.y - map_origin_y) / resolution;
        int robot_idx = robot_y * width + robot_x;   
        //human map cost     
        int robot_h_cost = friendlymap_ptr->data[robot_idx];
        if(robot_h_cost < lower_h_threshold)robot_h_cost = lower_h_threshold;
        if(robot_h_cost > upper_h_threshold)robot_h_cost = upper_h_threshold;
        double eva_cur_h = (double)(upper_h_threshold-robot_h_cost)/(double)(upper_h_threshold - lower_h_threshold);
        //obstacle map cost
        int robot_o_cost = obstaclemap_ptr->data[robot_idx];
        if(robot_o_cost < lower_o_threshold)robot_o_cost = lower_o_threshold;
        if(robot_o_cost > upper_o_threshold)robot_o_cost = upper_o_threshold;
        double eva_cur_o = (double)(upper_o_threshold-robot_o_cost)/(double)(upper_o_threshold - lower_o_threshold);
        
        // //global val for evaluarion
        // double eva_avg_h;
        // bool stop_count_flg = false;
        // bool temp_stop_flg = false;
        // bool run_flg = false;
        // ros::Time stop_time;
        // ros::Time start_time;
        // int run_cnt = 0;
        // int colli_h = 0; //friendly collision
        // bool colli_h_flg = false;
        // int total_time = 0;
        // output_date_flg = false;
        // double colli_h_time = 0;
        // ROS::Time colli_h_t_str;
        path_assessment::Assessment assess_msg; // 將 "your_package_name" 替換為你的封包名稱
        // 設定 Header
        assess_msg.header.stamp = ros::Time::now();

        ros::Time cur_time = ros::Time::now();
        //2. start to run
        if((robot_twist.linear.x>0.01 || robot_twist.linear.y>0.01) && !run_flg){  
            ROS_INFO("start");
            run_flg = true;
            start_time = cur_time;
            colli_h_cnt = 0;    //count for collision frames
            colli_h = 0;        //count for collision times
            colli_h_flg = false;
            output_date_flg = true;
        }

        //3. is running
        if(run_flg){
            if((robot_twist.linear.x>0.01 || robot_twist.linear.y>0.01)){
                stop_count_flg = false;
            }
            // ROS_INFO("running");
            total_time = (cur_time-start_time).toSec();
            
            //count of human collision 
            if(eva_cur_h < 1.0 ){ //set collsion value = 10
                // ROS_INFO("collide");
                colli_h_cnt++;
                // first time of collision
                if(colli_h_cnt==1){
                    eva_avg_h = eva_cur_h;
                }
                // other times of collision
                if(colli_h_cnt>=2){
                    eva_avg_h = eva_avg_h*(colli_h_cnt-1)/colli_h_cnt+eva_cur_h*1/colli_h_cnt;
                }
                //first of a collision peak
                if(!colli_h_flg){
                    colli_h_flg = true;
                    colli_h+=1;
                    colli_h_t_str = cur_time;
                }
            }
            if(eva_cur_h ==1.0 && colli_h_flg){
                colli_h_flg = false;
                colli_h_time += (cur_time - colli_h_t_str).toSec();

            }

            //count of obstcle collision 
            if(eva_cur_o < 1.0 ){ //set collsion value = 10
                // ROS_INFO("obstacle collide");
                colli_o_cnt++;
                // first time of collision
                if(colli_o_cnt==1){
                    eva_avg_o = eva_cur_o;
                }
                // other times of collision
                if(colli_o_cnt>=2){
                    eva_avg_o = eva_avg_o*(eva_cur_o-1)/eva_cur_o+eva_cur_o*1/eva_cur_o;
                }
                //first of a collision peak
                if(!colli_o_flg){
                    colli_o_flg = true;
                    colli_o+=1;
                    colli_o_t_str = cur_time;
                }
            }
            if(eva_cur_o ==1.0 && colli_o_flg){
                colli_o_flg = false;
                colli_o_time += (cur_time - colli_o_t_str).toSec();

            }
        }

        //4. temperarily stop count //stop_count_flg = false, temp_stop_flg = true
        if((robot_twist.linear.x<0.01&&robot_twist.linear.y<0.01) && (!stop_count_flg) ){ 
            // ROS_INFO("stop_count");
            stop_time = cur_time;
            stop_count_flg = true;
        }

        //5. stop //stop_count_flg = true, temp_stop_flg = true
        if((robot_twist.linear.x<0.01&&robot_twist.linear.y<0.01) && stop_count_flg 
            && ((cur_time - stop_time).toSec() > waiting_stop)){ 
                // output the path evaluation data
                if(output_date_flg){
                    ROS_INFO("stop");
                    total_time -= waiting_stop;
                    ROS_INFO("fin cur: %.2f idx: %d, avg: %.2f, t: %.2f, colli_t: %.2f, colli: %d"
                    ,eva_cur_h,friendlymap_ptr->data[robot_idx],eva_avg_h
                    ,total_time,colli_h_time, colli_h);
                    ROS_INFO("fin cur: %.2f idx: %d, avg: %.2f, t: %.2f,colli_t: %.2f, colli: %d"
                    ,eva_cur_o,obstaclemap_ptr->data[robot_idx],eva_avg_o
                    ,total_time,colli_o_time, colli_o);
                        output_date_flg = false;
                }
                run_flg = false;
                total_time = 0;
                colli_h_time = 0;
                colli_o_time = 0;
        }

        // ROS_INFO("mid cur: %.2f idx: %d, avg: %.2f, t: %.2f,colli_t: %.2f, colli: %d"
        // ,eva_cur_h,friendlymap_ptr->data[robot_idx],eva_avg_h
        // ,total_time,colli_h_time, colli_h);
        // ROS_INFO("mid cur: %.2f idx: %d, avg: %.2f, t: %.2f,colli_t: %.2f, colli: %d"
        // ,eva_cur_o,obstaclemap_ptr->data[robot_idx],eva_avg_o
        // ,total_time,colli_o_time, colli_o);


        // 設定 assessment 消息的其他欄位
        assess_msg.stop = false;
        assess_msg.t = total_time;
        assess_msg.robot_x = robot_pose.position.x / resolution;
        assess_msg.robot_y = robot_pose.position.y / resolution;

        assess_msg.eva_cur_h = eva_cur_h;
        assess_msg.idx_h = friendlymap_ptr->data[robot_idx];
        assess_msg.eva_avg_h = eva_avg_h;
        assess_msg.colli_t_h = colli_h_time;
        assess_msg.colli_h = colli_h;

        assess_msg.eva_cur_o = eva_cur_o;
        assess_msg.idx_o = obstaclemap_ptr->data[robot_idx];
        assess_msg.eva_avg_o = eva_avg_o;
        assess_msg.colli_t_o = colli_o_time;
        assess_msg.colli_o = colli_o;


        // 發布消息
        pub_assessment_.publish(assess_msg);
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