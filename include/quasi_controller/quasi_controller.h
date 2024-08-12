#ifndef QUASI_CONTROLLER_H
#define QUASI_CONTROLLER_H

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

#define QUASI_SITL_ENABLED true
#if QUASI_SITL_ENABLED
    #include <gazebo_msgs/LinkStates.h>
#endif

#define GAIN_MISMATCHED false

using namespace std;

/*============================== Message Definitions ==============================*/

geometry_msgs::PoseStamped current_local_pos;
geometry_msgs::PoseStamped pose;
geometry_msgs::TwistStamped load_vel;
geometry_msgs::TwistStamped current_local_vel;
geometry_msgs::Pose quadpose;
geometry_msgs::Pose pendpose;
geometry_msgs::Pose loadpose;
geometry_msgs::Twist quadtwist;
geometry_msgs::Twist loadtwist;
geometry_msgs::PoseStamped load_pose, load_pose0;
mavros_msgs::AttitudeTarget attitude;
mavros_msgs::State current_state;
double_sls_controller::PTStates PTState;
double_sls_controller::AttOut att_out;

/*============================== Template & Struct Definitions ==============================*/

template<typename T>
T saturate(T val, T min, T max) {
    return std::min(std::max(val, min), max);
}

template<typename T>
T sigmoidf(T x) {
    return x/(1+std::abs(x));
}

struct PendulumAngles {
    double alpha, beta; // roll(alpha) pitch(beta) yaw
}penangle,penangle2;

struct sls_state {
    double x, y, z, alpha, beta, vx, vy, vz, gamma_alpha, gamma_beta;
}sls_state1;


/*============================== Common ==============================*/
double PI = 3.1415926535;

/*============================== Bool ==============================*/

bool track_debug_enabled = false;
bool param_tuning_enabled = false;
bool offb_ground_debug_enabled = false;
bool gain_mismatched = false;
bool rate_target_enabled = true;

#if QUASI_SITL_ENABLED
    bool quasi_sitl_enabled = true;
#else
    bool quasi_sitl_enabled = false;
#endif

bool quad_only_enabled = true;


/*============================== SLS Controller ==============================*/

double dv[10] = {};
double controller_output[3] = {};
double K_POS_Z = 2.2361;
double K_POS_X = 3.1623;
double K_POS_Y = 3.1623;
double K_VEL_Z = 3.0777;
double K_VEL_X = 8.4827;
double K_VEL_Y = 8.4827;
double K_ACC_Z = 0;
double K_ACC_X = 18.7962; 
double K_ACC_Y = 18.7962; 
double K_JER_Z = 0;
double K_JER_X = 17.4399; 
double K_JER_Y = 17.4399; 

// double K_POS_Z = 3.0777;
// double K_POS_X = 17.4399; 
// double K_POS_Y = 17.4399; 
// double K_VEL_Z = 2.2361;
// double K_VEL_X = 18.7962;
// double K_VEL_Y = 18.7962;
// double K_ACC_Z = 0;
// double K_ACC_X = 8.4827; 
// double K_ACC_Y = 8.4827;
// double K_JER_Z = 0;
// double K_JER_X = 3.1623;
// double K_JER_Y = 3.1623;

// double K_POS_Z = 2.2361;
// double K_POS_X = 5.4772; 
// double K_POS_Y = 5.4772; 
// double K_VEL_Z = 3.0777;
// double K_VEL_X = 10.3913;
// double K_VEL_Y = 10.3913;
// double K_ACC_Z = 0;
// double K_ACC_X = 9.6746; 
// double K_ACC_Y = 9.6746;
// double K_JER_Z = 0;
// double K_JER_X = 4.9345;
// double K_JER_Y = 4.9345;

#if GAIN_MISMATCHED
    double Kv12[12] = {K_VEL_Z, K_JER_X, K_JER_Y, K_POS_Z, K_ACC_X, K_ACC_Y, 0, K_VEL_X, K_VEL_Y, 0, K_POS_X, K_POS_Y};
#else
    double Kv12[12] = {K_POS_Z, K_POS_X, K_POS_Y, K_VEL_Z, K_VEL_X, K_VEL_Y, K_ACC_Z, K_ACC_X, K_ACC_Y, K_JER_Z, K_JER_X, K_JER_Y};
#endif

/*============================== Quadrotor Controller ==============================*/

double Kv6[6] = {4.3166, 4.3166, 4.316, 3.1037, 3.1037, 3.1037};
double setpoint[6] = {0, 0, -1.25, 0, 0, 0};

/*============================== SLS Physics ==============================*/

#if QUASI_SITL_ENABLED
    double M_QUAD = 1.55; // Mass of UAV
    double M_LOAD = 0.25;  // Mass of Load
    double L_CABLE = 0.85;  // Length of Cable
    double GRAVITY = 9.81;   // Gravitational constant

    double thrust_0 = (M_QUAD + M_LOAD) * GRAVITY;          // total weight of UAV and the payload
    double thrust_0_q = M_QUAD * GRAVITY; 
    double thrust_norm_hover = 0.53;   // Normalized thrust when SLS is in hover
    double thrust_coeff = 10;          // scaling coefficient of total thrust
#else
    double M_QUAD = 1.625; // Mass of UAV
    double M_LOAD = 0.25;  // Mass of Load
    double L_CABLE = 0.85;  // Length of Cable
    double GRAVITY = 9.81;   // Gravitational constant
    double thrust_0 = (M_QUAD + M_LOAD) * GRAVITY;             // total weight of UAV and the payload
    double thrust_0_q = M_QUAD * GRAVITY; 
    double thrust_norm_hover = 0.58;    // Normalized thrust when SLS is in hover
    double thrust_coeff = 10.0;          // scaling coefficient of total thrust    
#endif


double Param[4] = {M_QUAD, M_LOAD, L_CABLE, GRAVITY};
double Setpoint[3] = {0, 0, -0.4};
double attctrl_tau_ = 0.3;
double diff_time = 0.01; 
double err_pend_pose = 0;
double t0 = 0;





/*============================== Helper Function Declarations ==============================*/

PendulumAngles ToPenAngles(double Lx,double Ly,double Lz);
void PT_state_pub(ros::Publisher &sls_state_pub);
void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void att_out_pub(ros::Publisher &att_con_pub, const double controller_output[3]);
void quasi_update_params(const ros::NodeHandle &nh);
void quasi_print_params(void);
void get_quad_states(void);
void get_pend_states(void);
void apply_outerloop_control(double Kv6[6], double setpoint[6]);


/*============================== Callback Function Declarations ==============================*/

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_get_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void loadpose_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
void timerCallback(const ros::TimerEvent&);
void attitude_target_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
void sls_state_cb(const double_sls_controller::PTStates::ConstPtr& msg);
#if QUASI_SITL_ENABLED
    void gazebo_state_cb(const gazebo_msgs::LinkStates::ConstPtr& msg);
#endif


#endif