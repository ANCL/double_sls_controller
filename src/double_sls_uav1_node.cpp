/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <dynamic_reconfigure/server.h>
#include <double_sls_controller/configConfig.h>
#include <double_sls_controller/double_sls_controller.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>
#include <double_sls_controller/DSlsState.h>
#include <double_sls_controller/DEAState.h>
#include <DEAController.h>

#define MIN_DELTA_TIME 1e-16
#define L 0.85
#define PI 3.1415926535

mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget attitude;
mavros_msgs::AttitudeTarget attitude_dea;
double_sls_controller::DSlsState state18;
double_sls_controller::DEAState dea_xi4;

double gazebo_last_called;
double controller_last_called;
void stateCb(const mavros_msgs::State::ConstPtr& msg);
void gazeboCb(const gazebo_msgs::LinkStates::ConstPtr& msg, ros::Publisher* attitude_setpoint_pub);
void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void getAttitudeTarget(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
void applyQuadController(double Kv6[6], double setpoint[6]);
void applyDEAController(double_sls_controller::DSlsState state18, double_sls_controller::DEAState &dea_xi4, const double dea_k[24], const double dea_param[4], const double ref[13]);
geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2);

/* Gazebo Index Matching */
bool gazebo_link_name_matched = false;
bool dea_enabled;
bool param_tuning_enabled = false;
bool force_saturation_enabled = true;
int uav0_link_index;
int uav1_link_index;
int pend0_link_index;
int pend1_link_index;
int load_link_index;
const char* link_name[5] = {
    "px4vision_0::base_link", 
    "px4vision_1::base_link",
    "slung_load::pendulum_0::base_link",
    "slung_load::pendulum_1::base_link",
    "slung_load::load::base_link"
    };

/* uav0 */
geometry_msgs::PoseStamped uav0_pose, uav0_pose_last;
geometry_msgs::TwistStamped uav0_twist, uav0_twist_last;
/* uav1 */
geometry_msgs::PoseStamped uav1_pose, uav1_pose_last;
geometry_msgs::TwistStamped uav1_twist, uav1_twist_last;
/* pend0 */
geometry_msgs::Vector3 pend0_q, pend0_q_last, pend0_q_dot;
geometry_msgs::Vector3 pend0_omega;
/* pend1 */
geometry_msgs::Vector3 pend1_q, pend1_q_last, pend1_q_dot;
geometry_msgs::Vector3 pend1_omega;
/* load */
geometry_msgs::PoseStamped load_pose, load_pose_last;
geometry_msgs::TwistStamped load_twist;
/* DEA Controller Output Force */
geometry_msgs::Vector3Stamped dea_force_1;
geometry_msgs::Vector3Stamped dea_force_2;

void pubDebugData(
    ros::Publisher &test_setpoint_pub, mavros_msgs::AttitudeTarget &attitude_dea,
    ros::Publisher &dsls_state_pub, double_sls_controller::DSlsState &state18,
    ros::Publisher &dea_state_pub, double_sls_controller::DEAState &dea_xi4,
    ros::Publisher &dea_force_1_pub,  geometry_msgs::Vector3Stamped &dea_force_1, 
    ros::Publisher &dea_force_2_pub,  geometry_msgs::Vector3Stamped &dea_force_2 
){
    test_setpoint_pub.publish(attitude_dea);
    dsls_state_pub.publish(state18);
    dea_state_pub.publish(dea_xi4);
    dea_force_1_pub.publish(dea_force_1);
    dea_force_2_pub.publish(dea_force_2);
}

double Kv6[6] = {4.3166, 4.3166, 4.316, 3.1037, 3.1037, 3.1037};
double setpoint[6] = {0.0, 0.8010, -5.6010, 0, 0, 0};
const double dea_param[4] = {0.25, 1.55,  L, 9.81};
// const double dea_ref[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1.0, 0, 90};
const double dea_ref[13] = {5, 0.5, 0, 0, 5, 0.5, 0, PI/2, 0, 0, -5.0, 0, 90};


int main(int argc, char **argv){

    /* ROS Node Utilities */
    ros::init(argc, argv, "double_sls_uav1_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("/uav1/mavros/state", 10, stateCb);    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> ("/uav1/mavros/setpoint_position/local", 10);
    ros::Publisher attitude_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/uav1/mavros/setpoint_raw/attitude", 10);
    ros::Publisher test_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/uav1/double_sls_controller/setpoint_raw/attitude", 10);
    ros::Publisher dsls_state_pub = nh.advertise<double_sls_controller::DSlsState> ("/uav1/double_sls_controller/dsls_state", 10);
    ros::Publisher dea_state_pub = nh.advertise<double_sls_controller::DEAState> ("/uav1/double_sls_controller/dea_state", 10);
    ros::Publisher dea_force_1_pub = nh.advertise<geometry_msgs::Vector3Stamped> ("/uav1/double_sls_controller/dea_force_1", 10);
    ros::Publisher dea_force_2_pub = nh.advertise<geometry_msgs::Vector3Stamped> ("/uav1/double_sls_controller/dea_force_2", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("/uav1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/uav1/mavros/set_mode");
    // #if SITL_ENABLED
        ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, boost::bind(gazeboCb, _1, &attitude_setpoint_pub));
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // Pole Assignment Gains
    const double dea_k1[4] = {0.4025,    2.1325,    4.0800,   3.3500};
    const double dea_k2[4] = {0.4025,    2.1325,    4.0800,   3.3500};
    const double dea_k3[4] = {24.0000,   50.0000,   35.0000,   10.0000};
    const double dea_k4[4] = {2.0000,    3.0000,         0,         0};
    const double dea_k5[4] = {2.0000,    3.0000,         0,         0};
    const double dea_k6[4] = {2.0000,    3.0000,         0,         0};
    // LQR Gains
    // const double dea_k1[4] = {0.3162,    1.1969,    2.1070,    2.0770};
    // const double dea_k2[4] = {0.3162,    1.1969,    2.1070,    2.0770};
    // const double dea_k3[4] = {0.3162,    1.1969,    2.1070,    2.0770};
    // const double dea_k4[4] = {0.3162,    0.8558,         0,         0};
    // const double dea_k5[4] = {0.3162,    0.8558,         0,         0};
    // const double dea_k6[4] = {0.3162,    0.8558,         0,         0};

    if(!param_tuning_enabled){
        for(int i = 0; i < 4; i++){
            dea_k[0 + i*6] = dea_k1[i];
            dea_k[1 + i*6] = dea_k2[i];
            dea_k[2 + i*6] = dea_k3[i];
            dea_k[3 + i*6] = dea_k4[i];
            dea_k[4 + i*6] = dea_k5[i];
            dea_k[5 + i*6] = dea_k6[i];
        }        
    }

    dea_xi4.dea_xi4[0] = -9.81;
    dea_xi4.dea_xi4[1] = -9.81*0.5;
    dea_xi4.dea_xi4[2] = 0;
    dea_xi4.dea_xi4[3] = 0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = -0.8;
    pose.pose.position.z = 2.0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time node_start_time = ros::Time::now();
    while(ros::ok()){

        nh.getParam("/double_sls_controller/dea_enabled", dea_enabled);
        nh.getParam("bool", gotime);
        
        // if(param_tuning_enabled){
        //     std::vector<double> dea_k_vec;
        //     nh.getParam("dea_k24", dea_k_vec);
        // }
        
        // for(size_t i=0; i < 24; ++i){
        //     dea_k[i] = dea_k_vec[i];
        // }

        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        /* Publish System Data for Debugging */
        pubDebugData(
            test_setpoint_pub, attitude_dea, 
            dsls_state_pub, state18, 
            dea_state_pub, dea_xi4,
            dea_force_1_pub, dea_force_1, 
            dea_force_2_pub, dea_force_2
            );

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void stateCb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void gazeboCb(const gazebo_msgs::LinkStates::ConstPtr& msg, ros::Publisher* attitude_setpoint_pub){
    /* Match links on the first call*/
    if(!gazebo_link_name_matched){
        ROS_INFO("[gazeboCb] Matching Gazebo Links");
        int temp_index[5];
        for(int i=0; i<23; i++){
            for(int j=0; j<5; j++){
                if(msg->name[i] == link_name[j]){
                    temp_index[j] = i;
                };
            }
        }
        uav0_link_index = temp_index[0];    ROS_INFO_STREAM("[gazeboCb] uav0_link_index=" << uav0_link_index);
        uav1_link_index = temp_index[1];    ROS_INFO_STREAM("[gazeboCb] uav1_link_index=" << uav1_link_index);
        pend0_link_index = temp_index[2];   ROS_INFO_STREAM("[gazeboCb] pend0_link_index=" << pend0_link_index);
        pend1_link_index = temp_index[3];   ROS_INFO_STREAM("[gazeboCb] pend0_link_index=" << pend1_link_index);
        load_link_index = temp_index[4];    ROS_INFO_STREAM("[gazeboCb] load_link_index=" << load_link_index);
        gazebo_link_name_matched = true;
        ROS_INFO("[gazeboCb] Matching Complete");
    }

    double diff_time; // = 0.5;
    diff_time = (ros::Time::now().toSec() - gazebo_last_called); // gives nan or inf sometimes
    // ROS_INFO_STREAM("diff_time:" << diff_time);
    gazebo_last_called = ros::Time::now().toSec();
    
    uav0_pose.pose = msg -> pose[uav0_link_index];
    uav0_twist.twist = msg -> twist[uav0_link_index];
    uav1_pose.pose = msg -> pose[uav1_link_index];
    uav1_twist.twist = msg -> twist[uav1_link_index];
    load_pose.pose = msg -> pose[load_link_index];
    load_twist.twist = msg -> twist[load_link_index];

    // coordinate transform
    uav0_pose.pose.position.y = -uav0_pose.pose.position.y; 
    uav0_pose.pose.position.z = -uav0_pose.pose.position.z; 

    uav1_pose.pose.position.y = -uav1_pose.pose.position.y; 
    uav1_pose.pose.position.z = -uav1_pose.pose.position.z; 

    uav0_twist.twist.linear.y = -uav0_twist.twist.linear.y;
    uav0_twist.twist.linear.z = -uav0_twist.twist.linear.z;

    uav1_twist.twist.linear.y = -uav1_twist.twist.linear.y;
    uav1_twist.twist.linear.z = -uav1_twist.twist.linear.z;

    load_pose.pose.position.y = -load_pose.pose.position.y; 
    load_pose.pose.position.z = -load_pose.pose.position.z;    

    load_twist.twist.linear.y = -load_twist.twist.linear.y;
    load_twist.twist.linear.z = -load_twist.twist.linear.z;

    // // coordinate transform: ENU -> NED
    // uav0_pose.pose.position.x = uav0_pose.pose.position.y; 
    // uav0_pose.pose.position.y = -uav0_pose.pose.position.x;
    // uav0_pose.pose.position.z = -uav0_pose.pose.position.z; 

    // uav1_pose.pose.position.x = uav1_pose.pose.position.y; 
    // uav1_pose.pose.position.y = -uav1_pose.pose.position.x; 
    // uav1_pose.pose.position.z = -uav1_pose.pose.position.z; 

    // uav0_twist.twist.linear.x = uav0_twist.twist.linear.y;
    // uav0_twist.twist.linear.y = -uav0_twist.twist.linear.x;
    // uav0_twist.twist.linear.z = -uav0_twist.twist.linear.z;

    // uav1_twist.twist.linear.x = uav1_twist.twist.linear.y;
    // uav1_twist.twist.linear.y = -uav1_twist.twist.linear.x;
    // uav1_twist.twist.linear.z = -uav1_twist.twist.linear.z;

    // load_pose.pose.position.x = load_pose.pose.position.y; 
    // load_pose.pose.position.y = -load_pose.pose.position.x; 
    // load_pose.pose.position.z = -load_pose.pose.position.z;    

    // load_twist.twist.linear.x = load_twist.twist.linear.y;
    // load_twist.twist.linear.y = -load_twist.twist.linear.x;
    // load_twist.twist.linear.z = -load_twist.twist.linear.z;

    
    // q1
    pend0_q.x = (load_pose.pose.position.x - uav0_pose.pose.position.x);
    pend0_q.y = (load_pose.pose.position.y - uav0_pose.pose.position.y);
    pend0_q.z = (load_pose.pose.position.z - uav0_pose.pose.position.z);
    double q1_norm = sqrt(pend0_q.x*pend0_q.x + pend0_q.y*pend0_q.y + pend0_q.z*pend0_q.z);
    pend0_q.x = pend0_q.x / q1_norm;
    pend0_q.y = pend0_q.y / q1_norm;
    pend0_q.z = pend0_q.z / q1_norm;

    // q2
    pend1_q.x = (load_pose.pose.position.x - uav1_pose.pose.position.x);
    pend1_q.y = (load_pose.pose.position.y - uav1_pose.pose.position.y);
    pend1_q.z = (load_pose.pose.position.z - uav1_pose.pose.position.z);
    double q2_norm = sqrt(pend1_q.x*pend1_q.x + pend1_q.y*pend1_q.y + pend1_q.z*pend1_q.z);
    pend1_q.x = pend1_q.x / q2_norm;
    pend1_q.y = pend1_q.y / q2_norm;
    pend1_q.z = pend1_q.z / q2_norm;

    if(diff_time >= MIN_DELTA_TIME){    
        // omega
        pend0_q_dot.x = (pend0_q.x - pend0_q_last.x) / diff_time;
        pend0_q_dot.y = (pend0_q.y - pend0_q_last.y) / diff_time;
        pend0_q_dot.z = (pend0_q.z - pend0_q_last.z) / diff_time;
        pend1_q_dot.x = (pend1_q.x - pend1_q_last.x) / diff_time;
        pend1_q_dot.y = (pend1_q.y - pend1_q_last.y) / diff_time;
        pend1_q_dot.z = (pend1_q.z - pend1_q_last.z) / diff_time;
        // next step
        pend0_q_last = pend0_q;
        pend1_q_last = pend1_q;
    }

    pend0_omega = crossProduct(pend0_q, pend0_q_dot);
    pend1_omega = crossProduct(pend1_q, pend1_q_dot);

    // ROS_INFO_STREAM("diff:");
    // ROS_INFO_STREAM("pend0_omega.x:" << pend0_omega.x);
    // ROS_INFO_STREAM("pend0_omega.y:" << pend0_omega.y);
    // ROS_INFO_STREAM("pend0_omega.z:" << pend0_omega.z);
    // ROS_INFO_STREAM("pend1_omega.x:" << pend1_omega.x);
    // ROS_INFO_STREAM("pend1_omega.y:" << pend1_omega.y);
    // ROS_INFO_STREAM("pend1_omega.z:" << pend1_omega.z);
  
    // Directly Use Gazebo omegas:
    pend0_omega.x = (msg -> twist[pend0_link_index]).angular.x;
    pend0_omega.y = -(msg -> twist[pend0_link_index]).angular.y;
    pend0_omega.z = -(msg -> twist[pend0_link_index]).angular.z;
    pend1_omega.x = (msg -> twist[pend1_link_index]).angular.x;
    pend1_omega.y = -(msg -> twist[pend1_link_index]).angular.y;
    pend1_omega.z = -(msg -> twist[pend1_link_index]).angular.z;

    // ROS_INFO_STREAM("gazebo:");
    // ROS_INFO_STREAM("pend0_omega.x:" << pend0_omega.x);
    // ROS_INFO_STREAM("pend0_omega.y:" << pend0_omega.y);
    // ROS_INFO_STREAM("pend0_omega.z:" << pend0_omega.z);
    // ROS_INFO_STREAM("pend1_omega.x:" << pend1_omega.x);
    // ROS_INFO_STREAM("pend1_omega.y:" << pend1_omega.y);
    // ROS_INFO_STREAM("pend1_omega.z:" << pend1_omega.z);  

    // system state msg
    state18.state18[0] = load_pose.pose.position.x;
    state18.state18[1] = load_pose.pose.position.y;
    state18.state18[2] = load_pose.pose.position.z;
    state18.state18[3] = pend0_q.x;
    state18.state18[4] = pend0_q.y;
    state18.state18[5] = pend0_q.z;
    state18.state18[6] = pend1_q.x;
    state18.state18[7] = pend1_q.y;
    state18.state18[8] = pend1_q.z;
    state18.state18[9] = load_twist.twist.linear.x;
    state18.state18[10] = load_twist.twist.linear.y;
    state18.state18[11] = load_twist.twist.linear.z; 
    state18.state18[12] = pend0_omega.x;
    state18.state18[13] = pend0_omega.y;
    state18.state18[14] = pend0_omega.z;
    state18.state18[15] = pend1_omega.x;
    state18.state18[16] = pend1_omega.y;
    state18.state18[17] = pend1_omega.z;   
    state18.header.stamp = ros::Time::now();

    //publisher
    if(gotime){
        applyDEAController(state18, dea_xi4, dea_k, dea_param, dea_ref);
        attitude.header.stamp = ros::Time::now();
        attitude_setpoint_pub->publish(attitude_dea); 
    }
    else{
        applyQuadController(Kv6, setpoint); 
        attitude.header.stamp = ros::Time::now();
        attitude_setpoint_pub->publish(attitude); 
        applyDEAController(state18, dea_xi4, dea_k, dea_param, dea_ref);
    }

}

void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
    //temp
    double attctrl_tau_ = 0.3;
    double thrust_norm_hover = 0.538;
    double thrust_coeff = 100;  
    double thrust_0 = 1.55*9.81;

    attitude.header.stamp = ros::Time::now();
    double roll, pitch, yaw, thrust;
    thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1]/thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);
    tf2::Quaternion attitude_target_q;
    attitude_target_q.setRPY(roll, pitch, yaw);
    attitude.orientation.x = attitude_target_q.getX();
    attitude.orientation.y = attitude_target_q.getY();
    attitude.orientation.z = attitude_target_q.getZ();
    attitude.orientation.w = attitude_target_q.getW();

    Eigen::Vector4d curr_att;
    Eigen::Vector4d ref_att;

    curr_att(0) = uav1_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
    curr_att(1) = uav1_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
    curr_att(2) = uav1_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
    curr_att(3) = uav1_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
    ref_att(0) = attitude_target_q.getW();
    ref_att(1) = attitude_target_q.getX();
    ref_att(2) = attitude_target_q.getY();
    ref_att(3) = attitude_target_q.getZ();

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); // ROS_INFO_STREAM("qe:" << qe);
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};
    desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);   

    attitude.body_rate.x = desired_rate_(0); // ROS_INFO_STREAM("body_rate_x:" << attitude.body_rate.x << " desired_rate_(0):" << desired_rate_(0));
    attitude.body_rate.y = desired_rate_(1); // ROS_INFO_STREAM("body_rate_y:" << attitude.body_rate.y << " desired_rate_(1):" << desired_rate_(1));
    attitude.body_rate.z = desired_rate_(2); // ROS_INFO_STREAM("body_rate_z:" << attitude.body_rate.z << " desired_rate_(2):" << desired_rate_(2));


    attitude.thrust = std::max(0.0, std::min(1.0, (thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
    attitude.type_mask = 128; //1|2|4;
}

void getAttitudeTarget(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
    //temp
    double attctrl_tau_ = 0.3;
    double thrust_norm_hover = 0.538;
    double thrust_coeff = 100;  
    double thrust_0 = (1.55 + 0.5*0.25 )*9.81;

    attitude.header.stamp = ros::Time::now();
    double roll, pitch, yaw, thrust;
    thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1]/thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);
    tf2::Quaternion attitude_target_q;
    attitude_target_q.setRPY(roll, pitch, yaw);
    attitude.orientation.x = attitude_target_q.getX();
    attitude.orientation.y = attitude_target_q.getY();
    attitude.orientation.z = attitude_target_q.getZ();
    attitude.orientation.w = attitude_target_q.getW();

    Eigen::Vector4d curr_att;
    Eigen::Vector4d ref_att;

    curr_att(0) = uav1_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
    curr_att(1) = uav1_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
    curr_att(2) = uav1_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
    curr_att(3) = uav1_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
    ref_att(0) = attitude_target_q.getW();
    ref_att(1) = attitude_target_q.getX();
    ref_att(2) = attitude_target_q.getY();
    ref_att(3) = attitude_target_q.getZ();

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); // ROS_INFO_STREAM("qe:" << qe);
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};
    desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);   

    attitude.body_rate.x = desired_rate_(0); // ROS_INFO_STREAM("body_rate_x:" << attitude.body_rate.x << " desired_rate_(0):" << desired_rate_(0));
    attitude.body_rate.y = desired_rate_(1); // ROS_INFO_STREAM("body_rate_y:" << attitude.body_rate.y << " desired_rate_(1):" << desired_rate_(1));
    attitude.body_rate.z = desired_rate_(2); // ROS_INFO_STREAM("body_rate_z:" << attitude.body_rate.z << " desired_rate_(2):" << desired_rate_(2));


    attitude.thrust = std::max(0.0, std::min(1.0, (thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
    attitude.type_mask = 128; //1|2|4;
}

geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
    geometry_msgs::Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y; 
    result.y = v1.z * v2.x - v1.x * v2.z; 
    result.z = v1.x * v2.y - v1.y * v2.x; 
    return result; 
}

void applyQuadController(double Kv6[6], double setpoint[6]){
    double M_QUAD = 1.55;
    double controller_output[3];
    controller_output[0] = (-Kv6[0] * (uav1_pose.pose.position.x - setpoint[0]) - Kv6[3] * (uav1_twist.twist.linear.x - setpoint[3]))*M_QUAD;
    controller_output[1] = (-Kv6[1] * (uav1_pose.pose.position.y - setpoint[1]) - Kv6[4] * (uav1_twist.twist.linear.y - setpoint[4]))*M_QUAD;
    controller_output[2] = (-Kv6[2] * (uav1_pose.pose.position.z - setpoint[2]) - Kv6[5] * (uav1_twist.twist.linear.z - setpoint[5]) - 9.81)*M_QUAD;

    // ROS_INFO_STREAM("controller_output[0]" << controller_output[0]);
    // ROS_INFO_STREAM("controller_output[1]" << controller_output[1]);
    // ROS_INFO_STREAM("controller_output[2]" << controller_output[2]);
    force_rate_convert(controller_output, attitude);
}

void applyDEAController(
    double_sls_controller::DSlsState state18, 
    double_sls_controller::DEAState &dea_xi4, 
    const double dea_k[24], 
    const double dea_param[4], 
    const double dea_ref[13]
    ){

    /* Getting Full State */
    double state22[22] = {};
    for(int i = 0; i < 22; i++){
        if(i < 18) state22[i] = state18.state18[i];
        else if(i < 22) state22[i] = dea_xi4.dea_xi4[i-18];
    }

    /* Apply Controller */
    double F1[3];
    double F2[3];
    double xi_dot[4];
    double sys_output[24];
    DEAController(state22, dea_k, dea_param, dea_ref, ros::Time::now().toSec(), F1, F2, xi_dot, sys_output);
    for(int i = 0; i<4; i++){
        dea_xi4.dea_xi_dot4[i] = xi_dot[i];
    }

    if(force_saturation_enabled){
        double max_force_x_1 = 0.1;
        double max_force_y_1 = 0.1;
        double max_force_z_1 = 17;
        if(F1[0]>max_force_x_1) F1[0] = max_force_x_1;
        else if(F1[0]<-max_force_x_1) F1[0] = -max_force_x_1;
        if(F1[1]>max_force_y_1) F1[1] = max_force_y_1;
        else if(F1[1]<-max_force_y_1) F1[1] = -max_force_y_1;
        if(F1[2]>max_force_z_1) F1[2] = max_force_z_1;
        else if(F1[2]<-max_force_z_1) F1[2] = -max_force_z_1; 
        
        double max_force_x_2 = 0.1;
        double max_force_y_2 = 0.1;
        double max_force_z_2 = 17;
        if(F2[0]>max_force_x_2) F2[0] = max_force_x_2;
        else if(F2[0]<-max_force_x_2) F2[0] = -max_force_x_2;
        if(F2[1]>max_force_y_2) F2[1] = max_force_y_2;
        else if(F2[1]<-max_force_y_2) F2[1] = -max_force_y_2;
        if(F2[2]>max_force_z_2) F2[2] = max_force_z_2;
        else if(F2[2]<-max_force_z_2) F2[2] = -max_force_z_2;    
    }


    /* Infill DEA Force Msg */
    dea_force_1.header.stamp = ros::Time::now();
    dea_force_1.vector.x = F1[0];
    dea_force_1.vector.y = F1[1];
    dea_force_1.vector.z = F1[2];
    dea_force_2.header.stamp = ros::Time::now();
    dea_force_2.vector.x = F2[0];
    dea_force_2.vector.y = F2[1];
    dea_force_2.vector.z = F2[2];    
    getAttitudeTarget(F2, attitude_dea);


    /* Controller State Integration */
    double diff_time;
    bool reset_flag = false;
    diff_time = (ros::Time::now().toSec() - controller_last_called);
    controller_last_called = ros::Time::now().toSec(); //works well, gives 0.02
    // if(gotime){
        for(int j = 0; j < 4; j ++){
            dea_xi4.header.stamp = ros::Time::now();
            dea_xi4.dea_xi4[j] += xi_dot[j]*diff_time; // Euler, should be replaced with RK4
            if(std::isnan(dea_xi4.dea_xi4[j])){
                reset_flag = true;
                break;
            } 
        } 
    // }

    if(reset_flag == true){
        dea_xi4.dea_xi4[0] = -9.81;
        dea_xi4.dea_xi4[1] = -9.81*0.5;
        dea_xi4.dea_xi4[2] = 0;
        dea_xi4.dea_xi4[3] = 0;
    }
    
}

