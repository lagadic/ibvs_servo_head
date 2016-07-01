#include <iostream>
#include <vector>
#include <algorithm>

#include <visp_naoqi/vpNaoqiGrabber.h>
#include <vpRomeoTkConfig.h>

#include "ibvs_servo_head.h"

ibvs_servo_head::ibvs_servo_head(ros::NodeHandle &nh)
{
    // read in config options
    n = nh;

    m_actualPoint_isInitialized = false;
    m_desiredPoint_isInitialized = false;
    m_cam_is_initialized = false;

    m_statusPointActual = 0;
//    m_statusPointDesired = 0;
    m_servo_time_init = 0;

    n.param( "frequency", freq, 100);
    n.param<std::string>("actualPointTopicName", actualPointTopicName, "/visp_blobs_tracker/object_position");
    n.param<std::string>("desiredPointTopicName", desiredPointTopicName, "/visp_blobs_tracker/object_des_position");
    n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "joint_state");
    n.param<std::string>("cameraParameterTopicName", cameraParameterTopicName, "/SR300/rgb/image_info");
    n.param<std::string>("statusPointActualTopicName", statusPointActualTopicName, "/visp_blobs_tracker/status");
//    n.param<std::string>("cameraName", cameraName, "/visp_blobs_tracker/status");
    n.param("desired_point_enable", m_desired_point_enable, false);
    n.param( "trunk_isEnable", m_trunk_isEnable, false );
    n.param( "use_realsense_rgb", m_use_realsense_rgb, false );

    // Initialize subscriber and publisher
    if (m_desired_point_enable)
        desiredPointSub = n.subscribe( desiredPointTopicName, 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &ibvs_servo_head::getDesiredPointCb, this, _1 ));
    actualPointSub = n.subscribe( actualPointTopicName, 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr &)>) boost::bind( &ibvs_servo_head::getActualPointCb, this, _1 ));
    cameraParameterSub = n.subscribe( cameraParameterTopicName, 1, (boost::function < void(const sensor_msgs::CameraInfo::ConstPtr&)>) boost::bind( &ibvs_servo_head::setupCameraParameterCb, this, _1 ));
    statusPointActualSub = n.subscribe ( statusPointActualTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &ibvs_servo_head::getStatusPointActualCb, this, _1 ));
    cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);

    m_chain_name = "Head";
    if (m_use_realsense_rgb)
        m_eMc = vpNaoqiGrabber::getExtrinsicCameraParameters("CameraDepth", vpCameraParameters::perspectiveProjWithoutDistortion);
    else
        m_eMc = vpNaoqiGrabber::getExtrinsicCameraParameters("CameraLeftEye", vpCameraParameters::perspectiveProjWithDistortion);
    ROS_INFO("Launch NaoqiRobotros node");
    robot.open();

    std::cout << "eMc: " << m_eMc << std::endl;

    if (m_trunk_isEnable)
        m_jointNames.push_back("TrunkYaw");
    std::vector<std::string> jointNames_head = robot.getBodyNames(m_chain_name);
    m_jointNames.insert(m_jointNames.end(),jointNames_head.begin(),jointNames_head.end());
    m_jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll

    std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
    std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

    m_jointNames.insert(m_jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
    m_jointNames_tot = m_jointNames;
    m_jointNames_tot.push_back(jointNamesREye.at(0));
    m_jointNames_tot.push_back(jointNamesREye.at(1));

    std::cout << m_jointNames_tot << std::endl;

    if (m_trunk_isEnable)
    {
        m_MAP_head.resize(7,6);
        for (unsigned int i = 0; i < 4 ; i++)
            m_MAP_head[i][i]= 1;
        m_MAP_head[5][4]= 1;
        m_MAP_head[6][5]= 1;
    }
    else
    {
        m_MAP_head.resize(6,5);
        for (unsigned int i = 0; i < 3 ; i++)
          m_MAP_head[i][i]= 1;
        m_MAP_head[4][3]= 1;
        m_MAP_head[5][4]= 1;

    }

    m_numJoints = m_jointNames.size();
    m_q.resize(m_numJoints);
    m_q_dot.resize(m_numJoints);
    m_q2_dot.resize(m_numJoints);
    m_q_dot_msg.velocity.resize(m_jointNames_tot.size());
    m_q_dot_msg.name = m_jointNames_tot;
    m_jointMin.resize(m_numJoints);
    m_jointMax.resize(m_numJoints);


    std::cout << "m_numJoints:" << m_numJoints << std::endl;
    std::cout << "m_jointNames:" << m_jointNames << std::endl;
    std::cout << "m_jointNames_tot:" << m_jointNames_tot << std::endl;

    //Get joint limits
    robot.getJointMinAndMax(m_jointNames, m_jointMin, m_jointMax);
    if (m_trunk_isEnable)
        m_jointMin[0]=vpMath::rad(-16.8);
    m_jointMin[m_jointNames.size()-2]=vpMath::rad(-16.8);
    m_jointMin[m_jointNames.size()-1]=vpMath::rad(-14.8);

    m_jointMax[m_jointNames.size()-2]= vpMath::rad(17.2);
    m_jointMax[m_jointNames.size()-1]= vpMath::rad(15.3);

    std::cout << "limit max:" << m_jointMax << std::endl;
    std::cout << "limit min:" << m_jointMin << std::endl;

    //Set the stiffness
    robot.setStiffness(m_jointNames_tot, 1.f);

}

ibvs_servo_head::~ibvs_servo_head(){

}


void ibvs_servo_head::spin()
{
    ros::Rate loop_rate(freq);
    while(ros::ok()){
        this->computeControlLaw();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ibvs_servo_head::computeControlLaw()
{
    try
    {

        if ( m_actualPoint_isInitialized && m_desiredPoint_isInitialized && m_cam_is_initialized  && m_statusPointActual /*&& m_statusPointDesired*/)
        {
            static bool first_time = true;
            if (first_time) {
                std::cout << "-- Start visual servoing of the arm" << std::endl;
                m_servo_time_init = vpTime::measureTimeSecond();
                first_time = false;
            }

            vpAdaptiveGain lambda(2, 1.5, 30);
            m_servo_head.setCameraParameters(m_cam_param);
            m_servo_head.setLambda(lambda);
            m_servo_head.setCurrentFeature(m_actualPoint ) ;
            m_servo_head.setDesiredFeature(m_desiredPoint ) ;
            // Create twist matrix from target Frame to Arm end-effector (WristPitch)
            if (m_use_realsense_rgb)
            {
                m_servo_head.set_eJe( robot.get_eJe("Head") );
            }
            else
            {
                if (m_trunk_isEnable)
                    m_servo_head.set_eJe( robot.get_eJe("LEye_t")* m_MAP_head );
                else
                    m_servo_head.set_eJe( robot.get_eJe("LEye")* m_MAP_head );
            }

            m_servo_head.set_cVe( vpVelocityTwistMatrix(m_eMc.inverse()) );

            //Compute velocities PBVS task
            m_q_dot =  m_servo_head.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

            m_q = robot.getPosition(m_jointNames);
            m_q2_dot  = m_servo_head.m_task_head.secondaryTaskJointLimitAvoidance(m_q, m_q_dot, m_jointMin, m_jointMax);

            // Add mirroring eyes
            vpColVector q_dot_tot = m_q_dot + m_q2_dot;
            q_dot_tot.stack(m_q_dot[m_q_dot.size()-2]);
            q_dot_tot.stack(m_q_dot[m_q_dot.size()-1]);
            std::cout << "q_dot_tot: " << q_dot_tot << std::endl;

            publishCmdVel(q_dot_tot);
        }
        else
        {
            vpColVector q_dot_zero(m_jointNames_tot.size(),0);
            publishCmdVel(q_dot_zero);
        }
    }
    catch(vpException &e) {
      std::cout << "Exception tracking detection: " << e.getStringMessage() << std::endl;
      m_actualPoint_isInitialized = false;

    }



}

void ibvs_servo_head::publishCmdVel(const vpColVector &q)
{
    for (int i = 0; i < q.size(); i++)
    {
        m_q_dot_msg.velocity[i] = q[i];
    }

    cmdVelPub.publish(m_q_dot_msg);

}


void ibvs_servo_head::getDesiredPointCb(const geometry_msgs::PointStamped::ConstPtr &desiredPoint)
{

    m_desiredPoint.set_u(desiredPoint->point.x);
    m_desiredPoint.set_v(desiredPoint->point.y);

    if ( !m_desiredPoint_isInitialized )
    {
        ROS_INFO("DesiredPose received");
        m_desiredPoint_isInitialized = true;
    }

}


void ibvs_servo_head::getActualPointCb(const geometry_msgs::PointStamped::ConstPtr &actualPoint)
{
    m_actualPoint.set_u(actualPoint->point.x);
    m_actualPoint.set_v(actualPoint->point.y);

    if ( !m_actualPoint_isInitialized )
    {
        ROS_INFO("ActualPoint received");
        m_actualPoint_isInitialized = true;
    }
}


void ibvs_servo_head::getStatusPointActualCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_statusPointActual = status->data;
}

void ibvs_servo_head::setupCameraParameterCb(const sensor_msgs::CameraInfo::ConstPtr& cam)
{

  if (! m_cam_is_initialized) {
    //init m_camera parameters
    m_cam_param = visp_bridge::toVispCameraParameters(*cam);

    m_cam_is_initialized = true;
    if (! m_desired_point_enable)
    {
        m_desiredPoint.set_u(cam->height/2);
        m_desiredPoint.set_v(cam->width/2);

        m_desiredPoint_isInitialized = true;
        ROS_INFO("DesiredPoint is by default the center of the image");
    }
    cameraParameterSub.shutdown();
  }
}


