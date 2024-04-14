#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cmath>
// Custom utils
#include "localmap_utils.hpp"

class GlobalMapInflator {
public:
    GlobalMapInflator() {
        // 初始化ROS節點
        ros::NodeHandle nh;

        //init
        ros::param::param<double>("~inflation_radius", inflation_radius, 0.2);
        // 訂閱/map主題，並指定回呼函數
        map_subscriber = nh.subscribe("/map", 1, &GlobalMapInflator::mapCallback, this);

        // 發布/inflated_map主題
        inflated_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1);
    }

    // 地圖訂閱回呼函數
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        global_map_ = msg;
        double resolution = msg->info.resolution;
        // printf("first\n");
        // Filter kernel generator
        localmap_utils::butterworth_filter_generate(inflation_kernel_, inflation_radius, 3, resolution, 100);
 
        received = true;
    }
    void inflateMap() {
        if (received) {
            // printf("a\n");
            double resolution = global_map_->info.resolution;
            int width = global_map_->info.width;
            int height = global_map_->info.height;
            inflated_ptr = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
            inflated_ptr->info.width = width;      // map_width --> x axis
            inflated_ptr->info.height = height;     // map_height --> y_axis
            inflated_ptr->info.resolution = resolution;
            inflated_ptr->info.origin.position.x = global_map_->info.origin.position.x;
            inflated_ptr->info.origin.position.y = global_map_->info.origin.position.y;
            inflated_ptr->info.origin.orientation.w = global_map_->info.origin.orientation.w;
            inflated_ptr->data.resize((width/resolution) * (height/resolution));
            inflated_ptr->header.frame_id = "map";   

            for(int idx = 0; idx < global_map_->data.size(); idx++) {
                if(global_map_->data[idx] == 100) {
                    // printf("in\n");
                    localmap_utils::apply_butterworth_filter(inflated_ptr, inflation_kernel_, idx, 100);
                }
            }

            inflated_map_publisher.publish(inflated_ptr);
        }
    }
private:
    ros::Subscriber map_subscriber;
    ros::Publisher inflated_map_publisher;
    nav_msgs::OccupancyGrid::ConstPtr global_map_;
    nav_msgs::OccupancyGrid::Ptr inflated_ptr;
    std::vector<std::vector<int8_t> > inflation_kernel_;
    double inflation_radius;
    bool received = false;
};

int main(int argc, char **argv) {
    // 初始化ROS節點
    ros::init(argc, argv, "global_map_inflator");

    // 創建GlobalMapInflator實例
    GlobalMapInflator inflator;

  ros::Rate loop_rate(1);

 int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    // printf("fds\n");
    loop_rate.sleep();
    inflator.inflateMap();
  }
    return 0;
}
