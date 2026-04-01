#include <ros/ros.h>
#include "fitplane_planner/map_manager.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_manager_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    fitplane_planner::MapManager map_manager(nh, nh_private);

    ros::spin();

    return 0;
} 