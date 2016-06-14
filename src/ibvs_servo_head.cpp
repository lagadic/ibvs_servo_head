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

    m_cMh_isInitialized = false;
    m_cMdh_isInitialized = false;

    m_statusPoseHand = 0;
    m_statusPoseDesired = 0;
    m_servo_time_init = 0;

    n.param( "frequency", freq, 100);
    n.param<std::string>("actualPoseTopicName", actualPointTopicName, "/visp_blobs_tracker/object_position");
    n.param<std::string>("desiredPoseTopicName", desiredPointTopicName, "/visp_blobs_tracker/object_des_position");
    n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "joint_state");
    n.param<std::string>("statusPoseHandTopicName", statusPoseHandTopicName, "/visp_blobs_tracker/status");
//    n.param<std::string>("opt_arm", m_opt_arm, "right");
    n.param( "trunk_isEnable", m_trunk_isEnable, false );

    // Initialize subscriber and publisher
//    if ( m_statusPoseDesired_isEnable )
//    {
//        n.param<std::string>("StatusPoseDesiredTopicName", statusPoseDesiredTopicName, "/visp_blobs_tracker/status2");
//        statusPoseDesiredSub = n.subscribe ( statusPoseDesiredTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &ibvs_servo_head::getStatusPoseDesiredCb, this, _1 ));
//    }
//    else
//        m_statusPoseDesired = 1;

    desiredPointSub = n.subscribe( desiredPointTopicName, 1, (boost::function < void(const geometry_msgs::PointConstPtr&)>) boost::bind( &ibvs_servo_head::getDesiredPointCb, this, _1 ));
    actualPointSub = n.subscribe( actualPointTopicName, 1, (boost::function < void(const geometry_msgs::PointConstPtr &)>) boost::bind( &ibvs_servo_head::getActualPointCb, this, _1 ));
//    statusPoseHandSub = n.subscribe ( statusPoseHandTopicName, 1, (boost::function < void(const std_msgs::Int8::ConstPtr  &)>) boost::bind( &ibvs_servo_head::getStatusPoseHandCb, this, _1 ));
    cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);

//    if (m_opt_arm == "right")
//        m_chain_name = "RArm";
//    else
    m_chain_name = "Head";

    std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
    std::string name_transform = "qrcode_M_e_" + m_chain_name;
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    if (pm.parse(oMe_Arm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
        ros::shutdown();
    }
    else
        std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm << std::endl;

    ROS_INFO("Launch NaoqiRobotros node");
    robot.open();

    std::vector<std::string> jointNames;
    if (m_trunk_isEnable)
        jointNames.push_back("TrunkYaw");
    std::vector<std::string> jointNames_head = robot.getBodyNames(m_chain_name);
    jointNames.insert(jointNames.end(),jointNames_head.begin(),jointNames_head.end());
    jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll

    std::vector<std::string> jointNamesLEye = robot.getBodyNames("LEye");
    std::vector<std::string> jointNamesREye = robot.getBodyNames("REye");

    jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
    m_jointNames_tot = jointNames;
    m_jointNames_tot.push_back(jointNamesREye.at(0));
    m_jointNames_tot.push_back(jointNamesREye.at(1));

    std::cout << m_jointNames_tot << std::endl;

    vpMatrix MAP_head(7,6);
    for (unsigned int i = 0; i < 4 ; i++)
      MAP_head[i][i]= 1;
    MAP_head[5][4]= 1;
    MAP_head[6][5]= 1;


    m_numJoints = m_jointNames_tot.size();
    m_q.resize(m_numJoints);
    m_q_dot.resize(m_numJoints);
    m_q2_dot.resize(m_numJoints);
    m_q_dot_msg.velocity.resize(m_numJoints);
    m_q_dot_msg.name = m_jointNames_tot;
    m_jointMin.resize(m_numJoints);
    m_jointMax.resize(m_numJoints);

    //Get joint limits
    robot.getJointMinAndMax(m_jointNames_tot, m_jointMin, m_jointMax);

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

    if ( m_cMh_isInitialized && m_cMdh_isInitialized  && m_statusPoseHand && m_statusPoseDesired)
    {
        static bool first_time = true;
        if (first_time) {
            std::cout << "-- Start visual servoing of the arm" << std::endl;
            m_servo_time_init = vpTime::measureTimeSecond();
            first_time = false;
        }

        vpAdaptiveGain lambda(0.8, 0.05, 8);
        m_servo_head.setLambda(lambda);
        m_servo_head.setCurrentFeature(m_actualPoint ) ;
        m_servo_head.setDesiredFeature(m_desiredPoint ) ;
        // Create twist matrix from target Frame to Arm end-effector (WristPitch)
        vpVelocityTwistMatrix oVe_LArm(oMe_Arm);
        m_servo_head.set_eJe(robot.get_eJe(m_chain_name));  //// ???????
        m_servo_head.set_cVe(oVe_LArm);                     //// ???????

        //Compute velocities PBVS task
        m_q_dot = - m_servo_head.computeControlLaw(vpTime::measureTimeSecond() - m_servo_time_init);

        m_q = robot.getPosition(m_jointNames_tot);
        m_q2_dot  = m_servo_head.m_task_head.secondaryTaskJointLimitAvoidance(m_q, m_q_dot, m_jointMin, m_jointMax);
        // Add mirroring eyes
        vpColVector q_dot_tot = m_q_dot + m_q2_dot;
        q_dot_tot.stack(m_q_dot[m_q_dot.size()-2]);
        q_dot_tot.stack(m_q_dot[m_q_dot.size()-1]);

        publishCmdVel(q_dot_tot);
    }
    else
    {
        vpColVector q_dot_zero(m_numJoints,0);
        publishCmdVel(q_dot_zero);
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


void ibvs_servo_head::getDesiredPointCb(const geometry_msgs::Point::ConstPtr &desiredPoint)
{

    m_desiredPoint.set_u(desiredPoint->x);  ///// ????
    m_desiredPoint.set_v(desiredPoint->y);  ///// ????

    if ( !m_cMdh_isInitialized )
    {
        ROS_INFO("DesiredPose received");
        m_cMdh_isInitialized = true;
    }

}


void ibvs_servo_head::getActualPointCb(const geometry_msgs::Point::ConstPtr &actualPoint)
{
    m_actualPoint.set_u(actualPoint->x);  ///// ????
    m_actualPoint.set_v(actualPoint->y);  ///// ????

    if ( !m_cMh_isInitialized )
    {
        ROS_INFO("ActualPose received");
        m_cMh_isInitialized = true;
    }
}


void ibvs_servo_head::getStatusPointHandCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_statusPoseHand = status->data;
}


void ibvs_servo_head::getStatusPointDesiredCb(const std_msgs::Int8::ConstPtr  &status)
{
    m_statusPoseDesired = status->data;
}




