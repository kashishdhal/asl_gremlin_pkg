/**
 * @brief Backstepping controller Node
 * @file backstepping_controller_node.cpp
 * @author Murali VNV <muralivnv@gmail.com>
 */
/*
 * Copyright (c) 2017, muralivnv
 *
 * This file is part of the asl_gremlin_package and subject to the license terms
 * in the top-level LICENSE file of the asl_gremlin_pkg repository.
 * https://github.com/muralivnv/asl_gremlin_pkg/blob/master/LICENSE
 */

#include <controller/BackSteppingController.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <asl_gremlin_msgs/MotorAngVel.h>
#include <asl_gremlin_msgs/VehicleState.h>
#include <asl_gremlin_msgs/RefTraj.h>
#include <asl_gremlin_pkg/SubscribeTopic.h>
#include <std_msgs/Bool.h>

#include <thread>
#include <chrono>

using namespace controller;
using namespace asl_gremlin_msgs;
using namespace asl_gremlin_pkg;

int main(int argc, char** argv)
{
    ros::init(argc, argv , "backstepping_controller"); 

    ros::NodeHandle ctrl_nh;
    
    std::string ref_traj_topic_name, act_state_topic, ang_vel_topic;

    if(!ctrl_nh.getParam("trajectory/publisher_topic",ref_traj_topic_name))
    { ref_traj_topic_name = "trajectory_generation/reference_trajectory"; }

    if(!ctrl_nh.getParam("state_feedback/feedback_selected",act_state_topic))
    { act_state_topic = "state_feedback/selected_feedback"; }

    if(!ctrl_nh.getParam("controller/bckstp_cmd_angular_vel",ang_vel_topic))
    { ang_vel_topic = "controller/bckstp_cmd_angular_vel"; }
    
    SubscribeTopic<asl_gremlin_msgs::RefTraj> ref_traj(ctrl_nh, ref_traj_topic_name);
    SubscribeTopic<asl_gremlin_msgs::VehicleState> act_state(ctrl_nh, act_state_topic);
  
    
    std::unique_ptr<ControllerBase<RefTraj, VehicleState>> controller = 
                            std::make_unique<BackSteppingController<RefTraj, VehicleState>>(ctrl_nh);

    asl_gremlin_pkg::SubscribeTopic<std_msgs::Bool> sim(ctrl_nh,"start_sim"); 

    ros::Publisher ang_vel_cmd = ctrl_nh.advertise<MotorAngVel>
                                                    (ang_vel_topic, 20); 
                                                    
     ////////////////////////////////////////
     ////Edited by Kashish
   // ros::Publisher  cmd_vel = ctrl_nh.advertise<std_msgs::Float32>("controller/backstepping_cmd_vel",20);
    ///End Edit                                               
    ////////////////////////////////////////
                                                    
    double rate = 10.0;
    if (!ctrl_nh.getParam("sim/rate", rate))
    {
        ROS_WARN("Unable access parameter /%s/sim/rate, setting rate as 10Hz",
                    ros::this_node::getNamespace().c_str());
    }
    ros::Rate loop_rate(rate);
    ros::spinOnce();

    ROS_INFO("\033[1;32mInitialized\033[0;m:= %s",ros::this_node::getName().c_str());
    bool initialized = false;
 //   double velocity_double; //Kashish
    while(ros::ok())
    {
        if ( (sim.get_data())->data )
        {
            if (!initialized)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                ROS_INFO("\033[1;32mStarted\033[0;m:= generating control commands");
                initialized = true;
            }
            controller->calculate_control_action(*(ref_traj.get_data()),
                                                *(act_state.get_data()));
        }
        else
        { controller->reset(); initialized = false;}
    
     ////////////////////////////////////////
     ////Edited by Kashish
     //double velocity_double = controller->get_control_action_cmd_vel();
   //  std_msgs::Float32 velocity_std;
   //  velocity_std.data = velocity_double;
   // cmd_vel.publish(velocity_std);    
    ///End Edit                                         
    ////////////////////////////////////////   
    
       ang_vel_cmd.publish(*(controller->get_control_action()));

       ros::spinOnce();
       loop_rate.sleep();
    }
    return EXIT_SUCCESS;
}
