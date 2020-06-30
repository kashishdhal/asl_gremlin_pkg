/**
 * @brief AngularVelocityToPWM definitions
 * @file OmegaToPWM.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <controller/OmegaToPWM.h>

using namespace controller;
using namespace utility_pkg::custom_algorithms;

OmegaToPWM::OmegaToPWM(ros::NodeHandle& nh)
{
	if (!nh.getParam("motor/omega_to_pwm/omega", omega_lookup_))
	{ 
		ROS_ERROR("Can't access parameter /%s/motor/omega_to_pwm/omega, shutting down", 
					ros::this_node::getNamespace().c_str()); 
		ros::shutdown();
	}

	if (!nh.getParam("motor/omega_to_pwm/pwm", pwm_lookup_))
	{ 
		ROS_ERROR("Can't access parameter /%s/motor/omega_to_pwm/pwm, shutting down", 
					ros::this_node::getNamespace().c_str()); 
		ros::shutdown();
	}

    if (omega_lookup_.size() != pwm_lookup_.size())
    {
        ROS_ERROR("Mismatch sizes, /%s/motor/omega_to_pwm/pwm.size() != /%s/motor/omega_to_pwm/omega.size()",
                   ros::this_node::getNamespace().c_str(), ros::this_node::getNamespace().c_str());
        ros::shutdown();
    }

    std::string ang_vel_topic;
    /* Kashish
    if(!nh.getParam("controller/cmd_angular_vel_topic", ang_vel_topic))
    {	ang_vel_topic = "controller/cmd_angular_vel"; }
*/
     if(!nh.getParam("controller/final_cmd_angular_vel_topic", ang_vel_topic))
    {	ang_vel_topic = "controller/final_cmd_angular_vel"; }
    
    
    pwm_cmd_ = new asl_gremlin_msgs::MotorPwm();
    omega_cmd_ = new asl_gremlin_pkg::SubscribeTopic<asl_gremlin_msgs::MotorAngVel>(nh, ang_vel_topic,20);
}

OmegaToPWM::~OmegaToPWM()
{
    delete pwm_cmd_;
    delete omega_cmd_;
}

asl_gremlin_msgs::MotorPwm*
OmegaToPWM::convert_omega_to_pwm()
{
    ros::spinOnce();

    pwm_cmd_->pwm_l = lookup_table(omega_lookup_, pwm_lookup_, (omega_cmd_->get_data())->wl );
    pwm_cmd_->pwm_r = lookup_table(omega_lookup_, pwm_lookup_, (omega_cmd_->get_data())->wr );

    pwm_cmd_->header = (omega_cmd_->get_data())->header;

   return pwm_cmd_;
}

