#ifndef DOUBLE_SLS_CONTROLLER_H
#define DOUBLE_SLS_CONTROLLER_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>
#include <cmath>
#include <iostream>
#include <cstddef>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <double_sls_controller/PTStates.h>
#include <double_sls_controller/AttOut.h>
#include <double_sls_controller/PendAngle.h>

#define SITL_ENABLED true
#if SITL_ENABLED
    #include <gazebo_msgs/LinkStates.h>
#endif


class DoubleSLSController{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber mav_state_sub_;
        ros::Publisher local_pos_pub_;
        ros::ServiceClient arming_client_;
        ros::ServiceClient set_mode_client_;
        // ros::Rate rate(20.0);
        ros::Timer cmdloop_timer_, statusloop_timer_;
        ros::Time last_request_, reference_request_now_, reference_request_last_;

        mavros_msgs::State current_state_;
        void mav_state_cb(const mavros_msgs::State::ConstPtr &msg);
        void statusloopCallback(const ros::TimerEvent &event);
        void pubDefaultSetpoint(void);

        // mavros_msgs::SetMode offb_set_mode;
        mavros_msgs::CommandBool arm_cmd_;
        geometry_msgs::PoseStamped pose;
        mavros_msgs::SetMode offb_set_mode;        
        enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;
    public:
        DoubleSLSController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~DoubleSLSController();
};


#endif