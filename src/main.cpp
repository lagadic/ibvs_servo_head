#include <ros/ros.h>

#include "ibvs_servo_head.h"
#include <visp_bridge/3dpose.h>
#include <visp3/core/vpHomogeneousMatrix.h>

//#include <visp_naoqi/vpNaoqiRobot.h>
//#include <sensor_msgs/JointState.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv, "pbvs_arm" );

  ros::NodeHandle n(std::string("~"));

  ibvs_servo_head *node = new ibvs_servo_head(n);

  node->spin();

  delete node;

  return 0;
}




