/**
 * @brief BackSteppingController header
 * @file BackSteppingController.h
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#ifndef _controller_BACKSTEPPINGCONTROLLER_H_
#define _controller_BACKSTEPPINGCONTROLLER_H_

#include "ControllerBase.h"
#include <array>
#include <algorithm>
#include <ros/ros.h>
#include <cmath>
#include <controller/controllerGainSetConfig.h>
#include "controller_utilities.h"
#include <dynamic_reconfigure/server.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <std_msgs/Float32.h>


using namespace controller;

#define SIGN(x) (x>0?1:(x==0?0:-1))

namespace controller{

template<typename BoundsType, typename DataType>
inline BoundsType saturate(BoundsType lb, DataType val, BoundsType ub) noexcept
{  return std::min(ub, std::max(lb, static_cast<BoundsType>(val)) ); }

template<typename ArgType>
inline ArgType calc_error_decay_rate(ArgType constant_gain, ArgType error) noexcept
{ return constant_gain*std::log(std::fabs( (error/0.1) + 0.01)); }


template<typename ref_state_type, typename act_state_type>
class BackSteppingController : 
                        public ControllerBase<ref_state_type, act_state_type>{

    public:
        BackSteppingController(ros::NodeHandle&);
        ~BackSteppingController(){
            delete wheel_angular_vel_;
        }

        double calculate_control_action(const ref_state_type&, const act_state_type&) override; //Kashish changed to double from void
        asl_gremlin_msgs::MotorAngVel* get_control_action() override;

        void reset(){
            wheel_angular_vel_->wl = 0.0;
            wheel_angular_vel_->wr = 0.0;
        }
        
       // double vel_cmd;

    private:
        int msg_count_ = 0;
        std::array<double,3> lambda_gains_{{0.2, 0.2, 5.0}};
        double lambda_x_ = 0.0, lambda_y_ = 0.0,
               lambda_theta_ = 5.0, lambda_thetaDot_ = 1.0;

        asl_gremlin_msgs::MotorAngVel* wheel_angular_vel_;

        double radius_of_wheel_ = 0.06858, vehicle_base_length_ = 0.3353,
               max_wheel_angular_vel_ = 12.5, vel_cmd=1; 
               

        dynamic_reconfigure::Server<controller::controllerGainSetConfig> dr_gain_srv_;
        dynamic_reconfigure::Server<controller::controllerGainSetConfig>::CallbackType fun_;
        void dynamic_reconfigure_gain_callback(controller::controllerGainSetConfig&, uint32_t);
};

template<typename ref_state_type, typename act_state_type>
BackSteppingController<ref_state_type, act_state_type>::BackSteppingController(ros::NodeHandle& nh)
{
    fun_ = boost::bind(&BackSteppingController::dynamic_reconfigure_gain_callback,
                        this, _1, _2);
    dr_gain_srv_.setCallback(fun_);

    auto nh_namespace = ros::this_node::getNamespace();
    if (!nh.getParam("wheel/radius",radius_of_wheel_))
    {
        ROS_WARN("Can't access parameter: /%s/wheel/radius, setting to 0.06858m",
                    nh_namespace.c_str());
    }

    if (!nh.getParam("chassis/base_length",vehicle_base_length_))
    {
        ROS_WARN("Can't access parameter: /%s/chassis/base_length, setting to 0.3353m",
                    nh_namespace.c_str());
    }

    if (!nh.getParam("wheel/max_angular_vel",max_wheel_angular_vel_))
    {
        ROS_WARN("Can't access parameter: /%s/wheel/max_angular_vel, setting to 12.5(rad/sec)",
                    nh_namespace.c_str());
    }

    wheel_angular_vel_ = new asl_gremlin_msgs::MotorAngVel();
    wheel_angular_vel_->wl = 0.0;
    wheel_angular_vel_->wr = 0.0;
}


template<typename ref_state_type, typename act_state_type>
double BackSteppingController<ref_state_type, act_state_type>::calculate_control_action(const ref_state_type& ref, const act_state_type& actual)
{
    double actual_hdg = actual.heading*M_PI/180.0;

    double error_x = actual.pose.point.x - ref.x;
    double error_y = actual.pose.point.y - ref.y;
    
    lambda_x_ = controller::calc_error_decay_rate(lambda_gains_[0], error_x);
    lambda_y_ = controller::calc_error_decay_rate(lambda_gains_[1], error_y);

    double x_act_dot_req = ref.x_dot  - lambda_x_*error_x;
    double y_act_dot_req = ref.y_dot  - lambda_y_*error_y;

    double theta_cmd = std::atan2(y_act_dot_req, x_act_dot_req);

    double error_theta = controller::delta_theta(actual_hdg, theta_cmd);
    lambda_theta_ = controller::calc_error_decay_rate(lambda_gains_[2], error_theta);

    double vel_cmd = std::sqrt( x_act_dot_req*x_act_dot_req + y_act_dot_req*y_act_dot_req );

    
    double angular_vel_sum = (2/radius_of_wheel_)*vel_cmd,
           angular_vel_diff = 0.0;

    if ( vel_cmd <= 0.2 )
    { angular_vel_diff = -lambda_theta_ * controller::delta_theta(actual_hdg, ref.theta); }

    else
    {
        double theta_dot_req = (1/vel_cmd)*(std::cos(theta_cmd)*(ref.y_ddot - lambda_y_*(vel_cmd*std::sin(actual_hdg) - ref.y_dot)) - 
                                            std::sin(theta_cmd)*(ref.x_ddot - lambda_x_*(vel_cmd*std::cos(actual_hdg) - ref.x_dot)));

        angular_vel_diff = (vehicle_base_length_/radius_of_wheel_)*(lambda_thetaDot_*theta_dot_req - lambda_theta_*error_theta);
    }

    if ( std::fabs(angular_vel_diff) > 40.0 )
    { angular_vel_diff = SIGN(angular_vel_diff)*40.0; }

    angular_vel_sum = std::min<double>(angular_vel_sum, 40.0);

    wheel_angular_vel_->wl = 0.5*(angular_vel_sum - angular_vel_diff);
    wheel_angular_vel_->wr = 0.5*(angular_vel_sum + angular_vel_diff);

    wheel_angular_vel_->wl = controller::saturate(-max_wheel_angular_vel_, wheel_angular_vel_->wl, max_wheel_angular_vel_);
    wheel_angular_vel_->wr = controller::saturate(-max_wheel_angular_vel_, wheel_angular_vel_->wr, max_wheel_angular_vel_);

    if( (ref.header.frame_id).compare("collision_avoidance") ==0 )
    {
    wheel_angular_vel_->header.frame_id  =  "collision_avoidance";
    }

    return vel_cmd;
}

template<typename ref_state_type, typename act_state_type>
asl_gremlin_msgs::MotorAngVel* BackSteppingController<ref_state_type, act_state_type>::get_control_action()
{
    wheel_angular_vel_->header.seq       =  msg_count_;
    wheel_angular_vel_->header.stamp     =  ros::Time::now();
    if( (wheel_angular_vel_->header.frame_id).compare("collision_avoidance") !=0 )
    {
    wheel_angular_vel_->header.frame_id  =  "wheel frame";
    }
    ++msg_count_;

    return wheel_angular_vel_;
}


template<typename ref_state_type, typename act_state_type>
void BackSteppingController<ref_state_type, act_state_type>::
    dynamic_reconfigure_gain_callback(controller::controllerGainSetConfig& config, uint32_t level)
{
    ROS_INFO("\033[0;33mUpdated\033[0;m:= {Control-Gains}-> \033[1;37mlambda_x\033[0;m:= %f,"
                " \033[1;37mlambda_y\033[0;m:= %f, \033[1;37mlambda_theta\033[0;m:= %f",
             config.lambda_x, config.lambda_y, config.lambda_theta);
    lambda_gains_[0] = config.lambda_x;
    lambda_gains_[1] = config.lambda_y;
    lambda_gains_[2] = config.lambda_theta;
}

} // end namespace {controller}


#endif
