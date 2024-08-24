/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <double_sls_controller/double_sls_controller.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>
#include <double_sls_controller/DSlsState.h>

mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget attitude;
double_sls_controller::DSlsState state18;
double gazebo_last_called;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
void apply_outerloop_control(double Kv6[6], double setpoint[6]);
void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude);
geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2);

/* Gazebo Index Matching */
bool gazebo_link_name_matched = false;
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
geometry_msgs::PoseStamped pend0_pose;
geometry_msgs::TwistStamped pend0_twist;
geometry_msgs::Vector3 pend0_q, pend0_q_last, pend0_q_dot;
geometry_msgs::Vector3 pend0_omega;
/* pend1 */
geometry_msgs::PoseStamped pend1_pose;
geometry_msgs::TwistStamped pend1_twist;
geometry_msgs::Vector3 pend1_q, pend1_q_last, pend1_q_dot;
geometry_msgs::Vector3 pend1_omega;
/* load */
geometry_msgs::PoseStamped load_pose, load_pose_last;
geometry_msgs::TwistStamped load_twist;

int main(int argc, char **argv){
    ros::init(argc, argv, "double_sls_uav0_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/uav0/mavros/state", 10, state_cb);    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav0/mavros/setpoint_position/local", 10);
    ros::Publisher attitude_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/uav0/mavros/setpoint_raw/attitude", 10);
    ros::Publisher dsls_state_pub = nh.advertise<double_sls_controller::DSlsState>
            ("/uav0/double_sls_controller/dsls_state", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav0/mavros/set_mode");
    // #if SITL_ENABLED
        ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, gazeboCallback);
    //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(50.0);
        
    double Kv6[6] = {4.3166, 4.3166, 4.316, 3.1037, 3.1037, 3.1037};
    double setpoint[6] = {0, -1.0, -2.5, 0, 0, 0};
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0.7;
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

    while(ros::ok()){
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

        // local_pos_pub.publish(pose);
        apply_outerloop_control(Kv6, setpoint);  
        attitude.header.stamp = ros::Time::now();
        attitude_setpoint_pub.publish(attitude); 
        dsls_state_pub.publish(state18);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
    /* Match links on the first call*/
    if(!gazebo_link_name_matched){
        ROS_INFO("[gazebo_cb] Matching Gazebo Links");
        int temp_index[5];
        for(int i=0; i<23; i++){
            for(int j=0; j<5; j++){
                if(msg->name[i] == link_name[j]){
                    temp_index[j] = i;
                };
            }
        }
        uav0_link_index = temp_index[0];    ROS_INFO_STREAM("[gazebo_cb] uav0_link_index=" << uav0_link_index);
        uav1_link_index = temp_index[1];    ROS_INFO_STREAM("[gazebo_cb] uav1_link_index=" << uav1_link_index);
        pend0_link_index = temp_index[2];   ROS_INFO_STREAM("[gazebo_cb] pend0_link_index=" << pend0_link_index);
        pend1_link_index = temp_index[3];   ROS_INFO_STREAM("[gazebo_cb] pend0_link_index=" << pend1_link_index);
        load_link_index = temp_index[4];    ROS_INFO_STREAM("[gazebo_cb] load_link_index=" << load_link_index);
        gazebo_link_name_matched = true;
        ROS_INFO("[gazebo_cb] Matching Complete");
    }

    uav0_pose.pose = msg -> pose[uav0_link_index];
    uav0_twist.twist = msg -> twist[uav0_link_index];
    uav1_pose.pose = msg -> pose[uav1_link_index];
    uav1_twist.twist = msg -> twist[uav1_link_index];
    // x__p
    load_pose.pose = msg -> pose[load_link_index];
    // q1
    pend0_q.x = load_pose.pose.position.x - uav0_pose.pose.position.x;
    pend0_q.y = load_pose.pose.position.y - uav0_pose.pose.position.y;
    pend0_q.z = load_pose.pose.position.z - uav0_pose.pose.position.z;
    // q2
    pend1_q.x = load_pose.pose.position.x - uav1_pose.pose.position.x;
    pend1_q.y = load_pose.pose.position.y - uav1_pose.pose.position.y;
    pend1_q.z = load_pose.pose.position.z - uav1_pose.pose.position.z;
    // v__p
    load_twist.twist = msg -> twist[load_link_index];
    // omega
    double diff_time; // = 0.5;
    diff_time = (ros::Time::now().toSec() - gazebo_last_called); // gives nan or inf sometimes
    gazebo_last_called = ros::Time::now().toSec();
    pend0_q_dot.x = (pend0_q.x - pend0_q_last.x) / diff_time;
    pend0_q_dot.y = (pend0_q.y - pend0_q_last.y) / diff_time;
    pend0_q_dot.z = (pend0_q.z - pend0_q_last.z) / diff_time;
    pend1_q_dot.x = (pend1_q.x - pend1_q_last.x) / diff_time;
    pend1_q_dot.y = (pend1_q.y - pend1_q_last.y) / diff_time;
    pend1_q_dot.z = (pend1_q.z - pend1_q_last.z) / diff_time;
    pend0_omega = crossProduct(pend0_q, pend0_q_dot);
    pend1_omega = crossProduct(pend1_q, pend1_q_dot);
    
    // system states
    state18.dsls_state[0] = load_pose.pose.position.x;
    state18.dsls_state[1] = load_pose.pose.position.y;
    state18.dsls_state[2] = load_pose.pose.position.z;
    state18.dsls_state[3] = pend0_q.x;
    state18.dsls_state[4] = pend0_q.y;
    state18.dsls_state[5] = pend0_q.z;
    state18.dsls_state[6] = pend1_q.x;
    state18.dsls_state[7] = pend1_q.y;
    state18.dsls_state[8] = pend1_q.z;
    state18.dsls_state[9] = load_twist.twist.linear.x;
    state18.dsls_state[10] = load_twist.twist.linear.y;
    state18.dsls_state[11] = load_twist.twist.linear.z; 
    state18.dsls_state[12] = pend0_omega.x;
    state18.dsls_state[13] = pend0_omega.y;
    state18.dsls_state[14] = pend0_omega.z;
    state18.dsls_state[15] = pend1_omega.x;
    state18.dsls_state[16] = pend1_omega.y;
    state18.dsls_state[17] = pend1_omega.z;   
    state18.header.stamp = ros::Time::now();

    // next step
    pend0_q_last = pend0_q;
    pend1_q_last = pend1_q;
}

void apply_outerloop_control(double Kv6[6], double setpoint[6]){
    double M_QUAD = 1.55;
    double controller_output[3];
    controller_output[0] = (-Kv6[0] * (uav0_pose.pose.position.x - setpoint[0]) - Kv6[3] * (uav0_twist.twist.linear.x - setpoint[3]))*M_QUAD;
    controller_output[1] = (-Kv6[1] * (-uav0_pose.pose.position.y - setpoint[1]) - Kv6[4] * (-uav0_twist.twist.linear.y - setpoint[4]))*M_QUAD;
    controller_output[2] = (-Kv6[2] * (-uav0_pose.pose.position.z - setpoint[2]) - Kv6[5] * (-uav0_twist.twist.linear.z - setpoint[5]) - 9.81)*M_QUAD;
    force_rate_convert(controller_output, attitude);
}

void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
    //temp
    double attctrl_tau_ = 0.3;
    double thrust_norm_hover = 0.53;
    double thrust_coeff = 10;  
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

    curr_att(0) = uav0_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
    curr_att(1) = uav0_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
    curr_att(2) = uav0_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
    curr_att(3) = uav0_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
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
    attitude.type_mask = 1|2|4;
}

geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
    geometry_msgs::Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y; 
    result.y = v1.z * v2.x - v1.x * v2.z; 
    result.z = v1.x * v2.y - v1.y * v2.x; 
    return result; 
}