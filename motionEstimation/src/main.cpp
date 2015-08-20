#include "ros/ros.h"
#include "motionEstimation_node.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "motionEstimation_node");
   motionEstimation_node me;
   ros::spin();
   return(0);
}
