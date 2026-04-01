#include "fitplane_planner/path_planner.h"

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    fitplane_planner::PathPlanner planner(nh, nh_private);

    ros::spin();

    return 0;
} 