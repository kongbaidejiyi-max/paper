#include <ros/ros.h>
#include "vector"
#include "backward.hpp"
#include <plane.h>
#include <clocale>

namespace backward
{
backward::SignalHandling sh;
}

/**
 * @brief Main function of the traversibility_mapping node
 * @param argc 
 * @param argv
 * @return int 
 */
int main(int argc, char** argv)
{
    std::setlocale(LC_ALL, ""); // Set locale to support UTF-8 characters in terminal output

    ros::init(argc, argv, "Traversibility_mapping");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Rate rate(2);
    FitPlane::World world(0.1, nh, nh_private);
    float plane_size = 0.3; // 0.5
    FitPlane::PlaneMap planemap(&world, plane_size);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();        
    }

    return 0;
}