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

#include <double_sls_controller/configConfig.h>
// #include <double_sls_controller/double_sls_controller.h>

#define SITL_ENABLED true
#if SITL_ENABLED
    #include <gazebo_msgs/LinkStates.h>
#endif

// /* SLS0 */
// geometry_msgs::PoseStamped uav0_pose, uav0_pose_last;
// geometry_msgs::TwistStamped uav0_twist;
// geometry_msgs::PoseStamped pend0_pose;
// geometry_msgs::TwistStamped pend0_twist;
// double_sls_controller::PendAngle pend0_angle;

// /* SLS1 */
// geometry_msgs::PoseStamped uav1_pose;
// geometry_msgs::TwistStamped uav1_twist;
// geometry_msgs::PoseStamped pend1_pose;
// geometry_msgs::TwistStamped pend1_twist;

/* Payload */
// geometry_msgs::PoseStamped load_pose, load_pose_last;
// geometry_msgs::TwistStamped load_vel;

#if SITL_ENABLED
    void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
#endif

bool gotime;

double dea_k[24];

void callback(double_sls_controller::configConfig &config, uint32_t level);

#endif


class doubleSlsCtrl {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private;
        double norm_thrust_const_, norm_thrust_offset_;
        double max_fb_acc_;    
        
    public:
        doubleSlsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~doubleSlsCtrl();
};