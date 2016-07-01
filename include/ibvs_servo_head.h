#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int8.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <visp_naoqi/vpNaoqiConfig.h>

#include <vpServoHead.h>

class ibvs_servo_head
{
public:

  ibvs_servo_head(ros::NodeHandle &nh);
  ~ibvs_servo_head();
  void computeControlLaw();
  void spin();
  void getActualPointCb(const geometry_msgs::PointStampedConstPtr &msg);
  void getDesiredPointCb(const geometry_msgs::PointStampedConstPtr &msg);
  void getStatusPointActualCb(const std_msgs::Int8::ConstPtr &status);
  void getStatusPointDesiredCb(const std_msgs::Int8::ConstPtr &status);
  void setupCameraParameterCb(const sensor_msgs::CameraInfo::ConstPtr &cam);
  void getRobotJoints();
  void publishCmdVel(const vpColVector &q);


protected:

  // Robot
  vpNaoqiRobot robot;
  std::vector<std::string> m_jointNames_tot;
  std::vector<std::string> m_jointNames;
  int m_numJoints;
  std::string m_chain_name;
  vpColVector m_jointMin;
  vpColVector m_jointMax;

  // ROS
  ros::NodeHandle n;
  std::string actualPointTopicName;
  std::string desiredPointTopicName;
  std::string statusPointActualTopicName;
  std::string statusPointDesiredTopicName;
  std::string cameraParameterTopicName;
  std::string cmdVelTopicName;
  std::string m_opt_arm;
  ros::Subscriber actualPointSub;
  ros::Subscriber desiredPointSub;
  ros::Subscriber cameraParameterSub;
  ros::Subscriber statusPointActualSub;
  ros::Publisher cmdVelPub;
  int freq;

  // Messages
  sensor_msgs::JointState m_q_dot_msg;

  //Servo Head
  vpServoHead m_servo_head;
  vpColVector m_q;
  vpColVector m_q_dot;
  vpColVector m_q2_dot;

  double m_servo_time_init;
  int m_statusPointActual;
  int m_statusPointDesired;

  vpImagePoint m_actualPoint;
  vpImagePoint m_desiredPoint;
  vpHomogeneousMatrix m_eMc;
//  vpHomogeneousMatrix m_cMdh;
  vpHomogeneousMatrix oMe_Arm;
  vpMatrix m_MAP_head;
  vpCameraParameters m_cam_param;

  //conditions
  bool m_actualPoint_isInitialized;
  bool m_desiredPoint_isInitialized;
  bool m_trunk_isEnable;
  bool m_desired_point_enable;
  bool m_cam_is_initialized;
  bool m_use_realsense_rgb;

};
