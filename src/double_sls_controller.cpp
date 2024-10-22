#include <double_sls_controller/double_sls_controller.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>
#include <DSLSDEAController.h>

dslsCtrl::dslsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private):nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE) {
    state_sub_0_ = nh_.subscribe<mavros_msgs::State> ("/uav0/mavros/state", 10, &dslsCtrl::stateCb_0, this, ros::TransportHints().tcpNoDelay());  
    state_sub_1_ = nh_.subscribe<mavros_msgs::State> ("/uav1/mavros/state", 10, &dslsCtrl::stateCb_1, this, ros::TransportHints().tcpNoDelay());   
    gazebo_state_sub_ = nh_.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1000, &dslsCtrl::gazeboCb, this, ros::TransportHints().tcpNoDelay());    
    local_pos_pub_0_ = nh_.advertise<geometry_msgs::PoseStamped> ("/uav0/mavros/setpoint_position/local", 10);
    local_pos_pub_1_ = nh_.advertise<geometry_msgs::PoseStamped> ("/uav1/mavros/setpoint_position/local", 10);
    attitude_setpoint_pub_0_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("/uav0/mavros/setpoint_raw/attitude", 10);
    attitude_setpoint_pub_1_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("/uav1/mavros/setpoint_raw/attitude", 10);
    test_setpoint_0_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("/double_sls_controller/setpoint_raw/attitude_0", 10);
    test_setpoint_1_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget> ("/double_sls_controller/setpoint_raw/attitude_1", 10);
    dea_force_0_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped> ("/double_sls_controller/dea_force_0", 10);
    dea_force_1_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped> ("/double_sls_controller/dea_force_1", 10);
    dsls_state_pub_ = nh_.advertise<double_sls_controller::DSlsState> ("/double_sls_controller/dsls_state", 10);
    lpf_data_pub_ = nh_.advertise<double_sls_controller::LPFData> ("/double_sls_controller/lpf_data", 10);
    dea_state_pub_ = nh_.advertise<double_sls_controller::DEAState> ("/double_sls_controller/dea_state", 10);    
    arming_client_0_ = nh_.serviceClient<mavros_msgs::CommandBool> ("/uav0/mavros/cmd/arming");
    arming_client_1_ = nh_.serviceClient<mavros_msgs::CommandBool> ("/uav1/mavros/cmd/arming");
    set_mode_client_0_ = nh_.serviceClient<mavros_msgs::SetMode> ("/uav0/mavros/set_mode");
    set_mode_client_1_ = nh_.serviceClient<mavros_msgs::SetMode> ("/uav1/mavros/set_mode");
    gz_link_state_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    set_ref_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("set_parameters", 10);
    
    /* Initialize ROS Parameters */
    // Mission
    nh_private_.param<bool>("mission_enabled", mission_enabled_, false); 
    // Inner-loop
    nh_private_.param<double>("throttle_offset", throttle_offset_, 0.0);
    nh_private_.param<double>("att_ctrl_tau", att_ctrl_tau_, 0.8);
    /* DEA Controller */
    // Switch
    nh_private_.param<bool>("dea_enabled", dea_enabled_, false); 
    nh_private_.param<bool>("dea_preintegrate_enabled", dea_preintegrate_enabled_, false);   
    // Reference
    nh_private_.param<double>("radium_scalar", radium_scalar_, 0.0); 
    nh_private_.param<double>("freq_scalar", freq_scalar_, 0.0);
    nh_private_.param<double>("radium_x", r_1_, 1.0);
    nh_private_.param<double>("radium_y", r_2_, 1.0);
    nh_private_.param<double>("radium_z", r_3_, 1.0);
    nh_private_.param<double>("freq_x", fr_1_, 1.0);
    nh_private_.param<double>("freq_y", fr_2_, 1.0);
    nh_private_.param<double>("freq_z", fr_3_, 1.0);
    nh_private_.param<double>("ref_x", c_1_, 0.0); 
    nh_private_.param<double>("ref_y", c_2_, 0.0);
    nh_private_.param<double>("ref_z", c_3_, -2.0);
    nh_private_.param<double>("phi_x", ph_1_, 0.0);
    nh_private_.param<double>("phi_y", ph_2_, PI/2);
    nh_private_.param<double>("phi_z", ph_3_, 0.0);
    nh_private_.param<double>("ref_q13", q_1_3_r_, 0.7406322196911044);
    nh_private_.param<double>("ref_q21", q_2_1_r_, 0);
    nh_private_.param<double>("ref_q22", q_2_2_r_, -0.6717825272800765);
    // Gains


    /* LPF */ 
    nh_private_.param<double>("lpf_tau", lpf_tau_, 0.01);
    nh_private_.param<double>("lpf_xi", lpf_xi_, 0.7);
    nh_private_.param<double>("lpf_omega", lpf_omega_, 100);



    
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.004), &dslsCtrl::cmdloopCallback, this);
    statusloop_timer_ = nh_.createTimer(ros::Duration(1), &dslsCtrl::statusloopCallback, this);
    
    if(!param_tuning_enabled_){
        for(int i = 0; i < 4; i++){
            dea_k_[0 + i*6] = dea_k1_[i];
            dea_k_[1 + i*6] = dea_k2_[i];
            dea_k_[2 + i*6] = dea_k3_[i];
            dea_k_[3 + i*6] = dea_k4_[i];
            dea_k_[4 + i*6] = dea_k5_[i];
            dea_k_[5 + i*6] = dea_k6_[i];
        }
    }

    for(int j=0; j<4; j++) dea_xi4_.dea_xi4[j] = dea_xi4_ic_[j];  

    // nh_.getParam("bool", dea_enabled_);
    dea_last_status_ = dea_enabled_;
}

dslsCtrl::~dslsCtrl() {
    // Destructor
}

void dslsCtrl::stateCb_0(const mavros_msgs::State::ConstPtr& msg){
    current_state_0_ = *msg;
}

void dslsCtrl::stateCb_1(const mavros_msgs::State::ConstPtr& msg){
    current_state_1_ = *msg;
}

void dslsCtrl::gazeboCb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    /* Match links on the first call*/
    if(!gazebo_link_name_matched_){
        ROS_INFO("[gazeboCb] Matching Gazebo Links");
        int temp_index[5];
        for(int i=0; i<23; i++){
            for(int j=0; j<5; j++){
                if(msg->name[i] == link_name_[j]){
                    temp_index[j] = i;
                };
            }
        }
        uav0_link_index_ = temp_index[0];    ROS_INFO_STREAM("[gazeboCb] uav0_link_index=" << uav0_link_index_);
        uav1_link_index_ = temp_index[1];    ROS_INFO_STREAM("[gazeboCb] uav1_link_index=" << uav1_link_index_);
        pend0_link_index_ = temp_index[2];   ROS_INFO_STREAM("[gazeboCb] pend0_link_index=" << pend0_link_index_);
        pend1_link_index_ = temp_index[3];   ROS_INFO_STREAM("[gazeboCb] pend0_link_index=" << pend1_link_index_);
        load_link_index_ = temp_index[4];    ROS_INFO_STREAM("[gazeboCb] load_link_index=" << load_link_index_);
        gazebo_link_name_matched_ = true;
        ROS_INFO("[gazeboCb] Matching Complete");
    }

    // ROS_INFO("[gazeboCb] Called");

    uav0_pose_.pose = msg -> pose[uav0_link_index_];
    uav0_twist_.twist = msg -> twist[uav0_link_index_];
    uav1_pose_.pose = msg -> pose[uav1_link_index_];
    uav1_twist_.twist = msg -> twist[uav1_link_index_];
    load_pose_.pose = msg -> pose[load_link_index_];
    load_twist_.twist = msg -> twist[load_link_index_];

    // coordinate transform
    if(use_ned_frame_) enu2ned();
    else enu2esd();
    
    // q1
    pend0_q_.x = (load_pose_.pose.position.x - uav0_pose_.pose.position.x);
    pend0_q_.y = (load_pose_.pose.position.y - uav0_pose_.pose.position.y);
    pend0_q_.z = (load_pose_.pose.position.z - uav0_pose_.pose.position.z);
    double q0_norm = sqrt(pend0_q_.x*pend0_q_.x + pend0_q_.y*pend0_q_.y + pend0_q_.z*pend0_q_.z);
    pend0_q_.x = pend0_q_.x / q0_norm;
    pend0_q_.y = pend0_q_.y / q0_norm;
    pend0_q_.z = pend0_q_.z / q0_norm;
    // q2
    pend1_q_.x = (load_pose_.pose.position.x - uav1_pose_.pose.position.x);
    pend1_q_.y = (load_pose_.pose.position.y - uav1_pose_.pose.position.y);
    pend1_q_.z = (load_pose_.pose.position.z - uav1_pose_.pose.position.z);
    double q1_norm = sqrt(pend1_q_.x*pend1_q_.x + pend1_q_.y*pend1_q_.y + pend1_q_.z*pend1_q_.z);
    pend1_q_.x = pend1_q_.x / q1_norm;
    pend1_q_.y = pend1_q_.y / q1_norm;
    pend1_q_.z = pend1_q_.z / q1_norm;

    double diff_time;
    diff_time = (ros::Time::now().toSec() - gazebo_last_called_); 
    gazebo_last_called_ = ros::Time::now().toSec();

    if(gazebo_omega_enabled_){
        pend0_omega_.x = (msg -> twist[pend0_link_index_]).angular.x;
        pend0_omega_.y = -(msg -> twist[pend0_link_index_]).angular.y;
        pend0_omega_.z = -(msg -> twist[pend0_link_index_]).angular.z;
        pend1_omega_.x = (msg -> twist[pend1_link_index_]).angular.x;
        pend1_omega_.y = -(msg -> twist[pend1_link_index_]).angular.y;
        pend1_omega_.z = -(msg -> twist[pend1_link_index_]).angular.z;
    }


    if(dea_enabled_ && dea_enabled_ != dea_last_status_) {
        dea_start_time_ = ros::Time::now().toSec();
        ROS_INFO_STREAM("[main] DEA controller enabled");
    }
    else if(!dea_enabled_ && dea_enabled_ != dea_last_status_){
        dea_end_time_ = ros::Time::now().toSec();
        ROS_INFO_STREAM("[main] DEA controller disabled");
    } 
    dea_last_status_ = dea_enabled_;
    // ROS_INFO_STREAM("dsls_dea_ref:" << dsls_dea_ref_[2] << " " << dsls_dea_ref_[6] << " " << dsls_dea_ref_[10]);

    executeMission(); // execute mission if enabled

    if(dea_enabled_){
        // ROS_INFO_STREAM("Running DEA...");
        if(open_loop_ctrl_enabled_) applyOpenLoopController();
        else applyDSLSDEAController(state18_, dea_xi4_, dea_k_, dea_param_, dsls_dea_ref_, ros::Time::now().toSec() - dea_start_time_);
        attitude_dea_0_.header.stamp = ros::Time::now();
        attitude_dea_1_.header.stamp = ros::Time::now();
        attitude_setpoint_pub_0_.publish(attitude_dea_0_); 
        attitude_setpoint_pub_1_.publish(attitude_dea_1_); 
    }
    else{
        // ROS_INFO_STREAM("Running Quad...");
        applyQuad0Controller(Kv6_, setpoint_0_); 
        applyQuad1Controller(Kv6_, setpoint_1_); 
        attitude_0_.header.stamp = ros::Time::now();
        attitude_1_.header.stamp = ros::Time::now();
        attitude_setpoint_pub_0_.publish(attitude_0_);
        attitude_setpoint_pub_1_.publish(attitude_1_);
        applyDSLSDEAController(state18_, dea_xi4_, dea_k_, dea_param_, dsls_dea_ref_, 0);
    }
    pubDebugData();
    
}

void dslsCtrl::force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav){
    double thrust_norm_hover = 0.538;
    double thrust_coeff = 100;  
    double thrust_0 = 1.55*9.81;
    double max_force = (uav_mass_ + 0.5 * load_mass_) * max_fb_acc_ * 2; //
    double force_norm = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);

    /* Attitude */
    
    // Getting Current Attitude
    geometry_msgs::Quaternion quat_msg;
    double curr_roll, curr_pitch, curr_yaw;    
    if(uav == 0) quat_msg = uav0_pose_.pose.orientation;
    else if(uav == 1) quat_msg = uav1_pose_.pose.orientation;
    tf2::Quaternion quat_tf;
    tf2::fromMsg(quat_msg, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(curr_roll, curr_pitch, curr_yaw);
    bool curr_rpy_debug_enabled = false;
    if(curr_rpy_debug_enabled) ROS_INFO_STREAM("Current RPY:" << curr_roll << " " << curr_pitch << " " << curr_yaw);

    // Clip reference force
    if(force_norm > max_force && force_clip_enabled_){
        // ROS_INFO_STREAM("[f2r] Force Trimming Detected, Norm: " << force_norm);
        for(int i=0; i<3; i++) controller_output[i] *= max_force / force_norm;
    }


    // Getting ENU Attitude Target (mavros will do the conversion to NED)
    attitude.header.stamp = ros::Time::now();
    double ref_roll, ref_pitch, ref_yaw, ref_thrust;
    ref_thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    ref_roll = std::asin((std::cos(curr_yaw) * controller_output[1] - std::sin(curr_yaw) * controller_output[0])/ref_thrust);
    ref_pitch = std::atan2(std::cos(curr_yaw) * controller_output[0] + std::sin(curr_yaw) * controller_output[1], -controller_output[2]);
    ref_yaw = 0;

    // Restrict Maximum Tilt
    if(std::abs(ref_roll) > max_tilt_angle_) ref_roll = std::copysign(ref_roll, max_tilt_angle_);
    if(std::abs(ref_pitch) > max_tilt_angle_) ref_pitch = std::copysign(ref_pitch, max_tilt_angle_);

    tf2::Quaternion attitude_target_q;
    bool ref_rpy_debug_enabled = false;
    if(ref_rpy_debug_enabled) ROS_INFO_STREAM("Target RPY:" << ref_roll << " " << ref_pitch << " " << ref_yaw);
    attitude_target_q.setRPY(ref_roll, ref_pitch, ref_yaw);
    attitude.orientation.x = attitude_target_q.getX();
    attitude.orientation.y = attitude_target_q.getY();
    attitude.orientation.z = attitude_target_q.getZ();
    attitude.orientation.w = attitude_target_q.getW();


    /* Rate */

    Eigen::Vector4d curr_att;
    Eigen::Vector4d ref_att;

    if (uav == 0){
        curr_att(0) = uav0_pose_.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose_.pose.ori.w:" << uav0_pose_.pose.orientation.w);
        curr_att(1) = uav0_pose_.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose_.pose.ori.x:" << uav0_pose_.pose.orientation.x);
        curr_att(2) = uav0_pose_.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose_.pose.ori.y:" << uav0_pose_.pose.orientation.y);
        curr_att(3) = uav0_pose_.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose_.pose.ori.z:" << uav0_pose_.pose.orientation.z);
    }
    else if (uav == 1){
        curr_att(0) = uav1_pose_.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose_.pose.ori.w:" << uav0_pose_.pose.orientation.w);
        curr_att(1) = uav1_pose_.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose_.pose.ori.x:" << uav0_pose_.pose.orientation.x);
        curr_att(2) = uav1_pose_.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose_.pose.ori.y:" << uav0_pose_.pose.orientation.y);
        curr_att(3) = uav1_pose_.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose_.pose.ori.z:" << uav0_pose_.pose.orientation.z);
    }
    ref_att(0) = attitude_target_q.getW();
    ref_att(1) = attitude_target_q.getX();
    ref_att(2) = attitude_target_q.getY();
    ref_att(3) = attitude_target_q.getZ();

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); // ROS_INFO_STREAM("qe:" << qe);
    attitude.body_rate.x = (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    attitude.body_rate.y = (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    attitude.body_rate.z = (2.0 / att_ctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);   

    // ROS_INFO_STREAM("body_rate_x:" << attitude.body_rate.x << " desired_rate_(0):" << desired_rate_(0));
    // ROS_INFO_STREAM("body_rate_y:" << attitude.body_rate.y << " desired_rate_(1):" << desired_rate_(1));
    // ROS_INFO_STREAM("body_rate_z:" << attitude.body_rate.z << " desired_rate_(2):" << desired_rate_(2));

    /* Thrust */

    // attitude.thrust = std::max(0.0, std::min(1.0, (ref_thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
    attitude.thrust = std::max(0.0, std::min(1.0, ref_thrust / (max_thrust_force_) + throttle_offset_));
    attitude.type_mask = 128;
    // attitude.type_mask = 1|2|4;
}

geometry_msgs::Vector3 dslsCtrl::crossProduct(const geometry_msgs::Vector3 v1, const geometry_msgs::Vector3 v2) {
    geometry_msgs::Vector3 result;
    result.x = v1.y * v2.z - v1.z * v2.y; 
    result.y = v1.z * v2.x - v1.x * v2.z; 
    result.z = v1.x * v2.y - v1.y * v2.x; 
    return result; 
}

void dslsCtrl::applyQuad0Controller(double Kv6[6], double setpoint[6]){
    double controller_output[3];
    controller_output[0] = (-Kv6[0] * (uav0_pose_.pose.position.x - setpoint[0]) - Kv6[3] * (uav0_twist_.twist.linear.x - setpoint[3]))*uav_mass_;
    controller_output[1] = (-Kv6[1] * (uav0_pose_.pose.position.y - setpoint[1]) - Kv6[4] * (uav0_twist_.twist.linear.y - setpoint[4]))*uav_mass_;
    controller_output[2] = (-Kv6[2] * (uav0_pose_.pose.position.z - setpoint[2]) - Kv6[5] * (uav0_twist_.twist.linear.z - setpoint[5]) - gravity_acc_)*uav_mass_;
    force_rate_convert(controller_output, attitude_0_, 0);
}

void dslsCtrl::applyQuad1Controller(double Kv6[6], double setpoint[6]){
    double controller_output[3];
    controller_output[0] = (-Kv6[0] * (uav1_pose_.pose.position.x - setpoint[0]) - Kv6[3] * (uav1_twist_.twist.linear.x - setpoint[3]))*uav_mass_;
    controller_output[1] = (-Kv6[1] * (uav1_pose_.pose.position.y - setpoint[1]) - Kv6[4] * (uav1_twist_.twist.linear.y - setpoint[4]))*uav_mass_;
    controller_output[2] = (-Kv6[2] * (uav1_pose_.pose.position.z - setpoint[2]) - Kv6[5] * (uav1_twist_.twist.linear.z - setpoint[5]) - gravity_acc_)*uav_mass_;
    force_rate_convert(controller_output, attitude_1_, 1);
}


int dslsCtrl::applyDSLSDEAController(
    double_sls_controller::DSlsState state18, 
    double_sls_controller::DEAState &dea_xi4, 
    const double dea_k[24], 
    const double dea_param[4], 
    const double dea_ref[15],
    double t
    ){

    /* Get Time Step */
    double diff_time;
    bool reset_controller_state = false;
    diff_time = (ros::Time::now().toSec() - controller_last_called_);
    // ROS_INFO_STREAM("diff_time:" << diff_time);
    controller_last_called_ = ros::Time::now().toSec(); 

    /* Finite Difference */
    if(diff_time > MIN_DELTA_TIME){    
        pend0_q_dot_ = applyFiniteDiffVector3(pend0_q_, pend0_q_last_, pend0_q_dot_last_, diff_time);
        pend1_q_dot_ = applyFiniteDiffVector3(pend1_q_, pend1_q_last_, pend0_q_dot_last_, diff_time);
        pend0_q_dot_lpf1_ = applyLPFVector3(pend0_q_dot_, pend0_q_dot_last_, pend0_q_dot_last_2_, pend0_q_dot_lpf1_last_, pend0_q_dot_lpf1_last_2_, diff_time, lpf_tau_, lpf_xi_, lpf_omega_, 1);
        pend1_q_dot_lpf1_ = applyLPFVector3(pend1_q_dot_, pend1_q_dot_last_, pend1_q_dot_last_2_, pend1_q_dot_lpf1_last_, pend1_q_dot_lpf1_last_2_, diff_time, lpf_tau_, lpf_xi_, lpf_omega_, 1);
        pend0_q_dot_lpf2_ = applyLPFVector3(pend0_q_dot_, pend0_q_dot_last_, pend0_q_dot_last_2_, pend0_q_dot_lpf2_last_, pend0_q_dot_lpf2_last_2_, diff_time, lpf_tau_, lpf_xi_, lpf_omega_, 2);
        pend1_q_dot_lpf2_ = applyLPFVector3(pend1_q_dot_, pend1_q_dot_last_, pend1_q_dot_last_2_, pend1_q_dot_lpf2_last_, pend1_q_dot_lpf2_last_2_, diff_time, lpf_tau_, lpf_xi_, lpf_omega_, 2);
    }    

    pend0_omega_ = crossProduct(pend0_q_, pend0_q_dot_);
    pend1_omega_ = crossProduct(pend1_q_, pend1_q_dot_);

    // LPF debug msg
    lpf_data_.header.stamp = ros::Time::now();
    lpf_data_.pend0_q_dot = pend0_q_dot_;
    lpf_data_.pend1_q_dot = pend1_q_dot_;
    lpf_data_.pend0_q_dot_lpf1 = pend0_q_dot_lpf1_;
    lpf_data_.pend1_q_dot_lpf1 = pend1_q_dot_lpf1_;
    lpf_data_.pend0_q_dot_lpf2 = pend0_q_dot_lpf2_;
    lpf_data_.pend1_q_dot_lpf2 = pend1_q_dot_lpf2_;      
    lpf_data_.pend0_omega_lpf1 = crossProduct(pend0_q_, pend0_q_dot_lpf1_);
    lpf_data_.pend1_omega_lpf1 = crossProduct(pend1_q_, pend1_q_dot_lpf1_);
    lpf_data_.pend0_omega_lpf2 = crossProduct(pend0_q_, pend0_q_dot_lpf2_);
    lpf_data_.pend1_omega_lpf2 = crossProduct(pend1_q_, pend1_q_dot_lpf2_);

    // next step
    pend0_q_last_2_ = pend0_q_last_;
    pend1_q_last_2_ = pend1_q_last_;

    pend0_q_last_ = pend0_q_;
    pend1_q_last_ = pend1_q_;

    pend0_q_dot_last_2_ = pend0_q_dot_last_;
    pend1_q_dot_last_2_ = pend1_q_dot_last_;
    pend0_q_dot_last_ = pend0_q_dot_;
    pend1_q_dot_last_ = pend1_q_dot_;

    pend0_q_dot_lpf1_last_2_ = pend0_q_dot_lpf1_last_;
    pend1_q_dot_lpf1_last_2_ = pend1_q_dot_lpf1_last_;
    pend0_q_dot_lpf2_last_2_ = pend0_q_dot_lpf2_last_;
    pend1_q_dot_lpf2_last_2_ = pend1_q_dot_lpf2_last_; 

    pend0_q_dot_lpf1_last_ = pend0_q_dot_lpf1_;
    pend1_q_dot_lpf1_last_ = pend1_q_dot_lpf1_;
    pend0_q_dot_lpf2_last_ = pend0_q_dot_lpf2_;   
    pend1_q_dot_lpf2_last_ = pend1_q_dot_lpf2_;       


    // system state msg
    if(time_sync_debug_enabled_) ROS_INFO_STREAM("[applyDSLSDEA] Time:" << ros::Time::now());
    state18_.header.stamp = ros::Time::now();
    state18_.state18[0] = load_pose_.pose.position.x;
    state18_.state18[1] = load_pose_.pose.position.y;
    state18_.state18[2] = load_pose_.pose.position.z;
    state18_.state18[3] = pend0_q_.x;
    state18_.state18[4] = pend0_q_.y;
    state18_.state18[5] = pend0_q_.z;
    state18_.state18[6] = pend1_q_.x;
    state18_.state18[7] = pend1_q_.y;
    state18_.state18[8] = pend1_q_.z;
    state18_.state18[9] = load_twist_.twist.linear.x;
    state18_.state18[10] = load_twist_.twist.linear.y;
    state18_.state18[11] = load_twist_.twist.linear.z; 
    state18_.state18[12] = pend0_omega_.x;
    state18_.state18[13] = pend0_omega_.y;
    state18_.state18[14] = pend0_omega_.z;
    state18_.state18[15] = pend1_omega_.x;
    state18_.state18[16] = pend1_omega_.y;
    state18_.state18[17] = pend1_omega_.z;   

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
    test(5);
    DSLSDEAController(state22, dea_k, dea_param, dea_ref, t, F1, F2, xi_dot);
    dea_xi4.header.stamp = ros::Time::now();
    for(int i = 0; i < 4; i ++){
        dea_xi4.dea_xi_dot4[i] = xi_dot[i];
    }

    /* Infill DEA Force Msg */
    dea_force_0_.header.stamp = ros::Time::now();
    dea_force_0_.vector.x = F1[0];
    dea_force_0_.vector.y = F1[1];
    dea_force_0_.vector.z = F1[2];

    dea_force_1_.header.stamp = ros::Time::now();
    dea_force_1_.vector.x = F2[0];
    dea_force_1_.vector.y = F2[1];
    dea_force_1_.vector.z = F2[2];

    force_rate_convert(F1, attitude_dea_0_, 0);
    force_rate_convert(F2, attitude_dea_1_, 1);

    /* Controller State Integration */

    if(dea_enabled_ || dea_preintegrate_enabled_){
        for(int j = 0; j < 4; j ++){
            dea_xi4_.dea_xi4[j] += xi_dot[j]*diff_time; // Euler
            if(dea_xi4.dea_xi4[j] > max_xi_value_ || std::isnan(dea_xi4.dea_xi4[j])){
                reset_controller_state = true;
                break;
            } 
        }         
    }
 
    if(reset_controller_state){
        for(int i = 0; i<4; i++) dea_xi4.dea_xi4[i] = dea_xi4_ic_[i];
        ROS_INFO_STREAM("[applyDSLSDEA] xi reset detected");
    }      

    return 0;
}


int dslsCtrl::enu2ned(void){
    uav0_pose_.pose.position.x = uav0_pose_.pose.position.y;
    uav0_pose_.pose.position.y = uav0_pose_.pose.position.x; 
    uav0_pose_.pose.position.z = -uav0_pose_.pose.position.z; 

    uav1_pose_.pose.position.x = uav1_pose_.pose.position.y; 
    uav1_pose_.pose.position.y = uav1_pose_.pose.position.x; 
    uav1_pose_.pose.position.z = -uav1_pose_.pose.position.z; 

    uav0_twist_.twist.linear.x = uav0_twist_.twist.linear.y;
    uav0_twist_.twist.linear.y = uav0_twist_.twist.linear.x;
    uav0_twist_.twist.linear.z = -uav0_twist_.twist.linear.z;

    uav1_twist_.twist.linear.x = uav1_twist_.twist.linear.y;
    uav1_twist_.twist.linear.y = uav1_twist_.twist.linear.x;
    uav1_twist_.twist.linear.z = -uav1_twist_.twist.linear.z;

    load_pose_.pose.position.x = load_pose_.pose.position.y; 
    load_pose_.pose.position.y = load_pose_.pose.position.x; 
    load_pose_.pose.position.z = -load_pose_.pose.position.z;    

    load_twist_.twist.linear.x = load_twist_.twist.linear.y;
    load_twist_.twist.linear.y = load_twist_.twist.linear.x;
    load_twist_.twist.linear.z = -load_twist_.twist.linear.z;

    return 0;
}

int dslsCtrl::enu2esd(void){
    uav0_pose_.pose.position.y = -uav0_pose_.pose.position.y; 
    uav0_pose_.pose.position.z = -uav0_pose_.pose.position.z; 

    uav1_pose_.pose.position.y = -uav1_pose_.pose.position.y; 
    uav1_pose_.pose.position.z = -uav1_pose_.pose.position.z; 

    uav0_twist_.twist.linear.y = -uav0_twist_.twist.linear.y;
    uav0_twist_.twist.linear.z = -uav0_twist_.twist.linear.z;

    uav1_twist_.twist.linear.y = -uav1_twist_.twist.linear.y;
    uav1_twist_.twist.linear.z = -uav1_twist_.twist.linear.z;

    load_pose_.pose.position.y = -load_pose_.pose.position.y; 
    load_pose_.pose.position.z = -load_pose_.pose.position.z;    

    load_twist_.twist.linear.y = -load_twist_.twist.linear.y;
    load_twist_.twist.linear.z = -load_twist_.twist.linear.z;

    return 0;
}

int dslsCtrl::applyOpenLoopController(void){
    
    return 0;
}

double dslsCtrl::applyFiniteDiff(double x, double x_last, double x_dot_last, double diff_time){
    bool fd_debug_enabled = false;
    double x_dot;
    if(diff_time < DBL_MIN) {
        printf("[FiniteDiff][Fatal]: Time Step Too Small\n");
        return x_dot_last;
    }

    else if(x - x_last == 0){
        printf("f\n");
        return x_dot_last;
    }
    else{
        x_dot = (x - x_last) / diff_time;
        if(fd_debug_enabled) printf("[FiniteDiff] Finite Difference Complete\n");
        return x_dot;
    }
}

geometry_msgs::Vector3 dslsCtrl::applyFiniteDiffVector3(geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, geometry_msgs::Vector3 v_dot_last, double diff_time){
    geometry_msgs::Vector3 v_dot;
    v_dot.x = applyFiniteDiff(v.x, v_last.x, v_dot_last.x, diff_time);
    v_dot.y = applyFiniteDiff(v.y, v_last.y, v_dot_last.y, diff_time);
    v_dot.z = applyFiniteDiff(v.z, v_last.z, v_dot_last.z, diff_time);
    return v_dot;
}

double dslsCtrl::applyLPF(double u, double u_last, double u_last_2, double y_last, double y_last_2, double Ts, double tau, double xi, double omega, int order){
    double y; // y(k)
    if(order == 1){ // 1st Order LPF
        double a = 2 / (Ts + 2 * tau);
        double b = (2 * tau - Ts) / (Ts + 2 * tau);
        y = a * (u - u_last) + b * y_last;
        if(lpf_debug_enabled_) printf("[LPF] 1st Order LPF Complete\n");
        return y;
    }
    else if(order == 2){ // 2nd Order LPF
        double k1 = 4 + 4 * xi * omega * Ts + omega * omega * Ts * Ts;
        double k2 = -8 + 2 * omega * omega * Ts * Ts;
        double k3 = 4 - 4 * xi * omega * Ts + omega * omega * Ts * Ts;
        double k4 = 2 * omega * omega * Ts;
        y = k4 / k1 * (u - u_last_2) - k2 / k1 * y_last - k3 / k1 * y_last_2;
        if(lpf_debug_enabled_) printf("[LPF] 2nd Order LPF Complete\n");
        return y;
    }
    else {
        printf("[LPF][Error] Order Error Detected\n");
    }
    return 1;
}

geometry_msgs::Vector3 dslsCtrl::applyLPFVector3(
    geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, geometry_msgs::Vector3 v_last_2, geometry_msgs::Vector3 v_dot_last, geometry_msgs::Vector3 v_dot_last_2, 
    double diff_time, double tau, double xi, double omega, int order
    ){
    geometry_msgs::Vector3 v_dot;
    v_dot.x = applyLPF(v.x, v_last.x, v_last_2.x, v_dot_last.x, v_dot_last_2.x, diff_time, tau, xi, omega, order);
    v_dot.y = applyLPF(v.x, v_last.y, v_last_2.y, v_dot_last.y, v_dot_last_2.y, diff_time, tau, xi, omega, order);
    v_dot.z = applyLPF(v.x, v_last.z, v_last_2.z, v_dot_last.z, v_dot_last_2.z, diff_time, tau, xi, omega, order);
    return v_dot;
}


void dslsCtrl::cmdloopCallback(const ros::TimerEvent &event) {
    if(time_sync_debug_enabled_) ROS_INFO_STREAM("[cmdloopCallback] Time:" << ros::Time::now());
    switch (node_state) {
    case WAITING_FOR_HOME_POSE:
        // waitForPredicate(&received_home_pose, "Waiting for home pose...");
        // ROS_INFO("Got pose! Drone Ready to be armed.");
        node_state = MISSION_EXECUTION;
        break;

    case MISSION_EXECUTION: {
        // Eigen::Vector3d desired_acc;
        // if (feedthrough_enable_) {
        // desired_acc = targetAcc_;
        // } else {
        // desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
        // }
        // computeBodyRateCmd(cmdBodyRate_, desired_acc);
        // pubReferencePose(targetPos_, q_des);
        // pubRateCommands(cmdBodyRate_, q_des);
        // appendPoseHistory();
        // pubPoseHistory();

        // double time = ros::Time::now().toSec();
        // if(gz_link_state_client_.exists()) {
        //     ROS_INFO_STREAM("link state exists");
        //     gz_link_state_request_.request.link_name = string("px4vision_0::base_link");
        //     gz_link_state_request_.request.reference_frame = string("world");
        //     gz_link_state_client_.call(gz_link_state_request_);
        // }

        // getLinkState(uav0_link_name_);
        // getLinkState(uav1_link_name_);
        // getLinkState(pend0_link_name_);
        // getLinkState(pend1_link_name_);
        // getLinkState(load_link_name_);
        // double srv_time = ros::Time::now().toSec() - time;
        // printf("srv_time: %.8f\n", srv_time);


        // nh_.getParam("bool", dea_enabled_);
        // if(dea_enabled_ && dea_enabled_ != dea_last_status_) {
        //     dea_start_time_ = ros::Time::now().toSec();
        //     ROS_INFO_STREAM("[main] DEA controller enabled");
        // }
        // else if(!dea_enabled_ && dea_enabled_ != dea_last_status_){
        //     dea_end_time_ = ros::Time::now().toSec();
        //     ROS_INFO_STREAM("[main] DEA controller disabled");
        // } 
        // dea_last_status_ = dea_enabled_;
        // // ROS_INFO_STREAM("dsls_dea_ref:" << dsls_dea_ref_[2] << " " << dsls_dea_ref_[6] << " " << dsls_dea_ref_[10]);

        // if(dea_enabled_){
        //     // ROS_INFO_STREAM("Running DEA...");
        //     if(open_loop_ctrl_enabled_) applyOpenLoopController();
        //     else applyDSLSDEAController(state18_, dea_xi4_, dea_k_, dea_param_, dsls_dea_ref_, ros::Time::now().toSec() - dea_start_time_);
        //     attitude_dea_0_.header.stamp = ros::Time::now();
        //     attitude_dea_1_.header.stamp = ros::Time::now();
        //     attitude_setpoint_pub_0_.publish(attitude_dea_0_); 
        //     attitude_setpoint_pub_1_.publish(attitude_dea_1_); 
        // }
        // else{
        //     // ROS_INFO_STREAM("Running Quad...");
        //     applyQuad0Controller(Kv6_, setpoint_0_); 
        //     applyQuad1Controller(Kv6_, setpoint_1_); 
        //     attitude_0_.header.stamp = ros::Time::now();
        //     attitude_1_.header.stamp = ros::Time::now();
        //     attitude_setpoint_pub_0_.publish(attitude_0_);
        //     attitude_setpoint_pub_1_.publish(attitude_1_);
        //     applyDSLSDEAController(state18_, dea_xi4_, dea_k_, dea_param_, dsls_dea_ref_, 0);
        // }
        // pubDebugData();

        break;
    }

    case LANDING: {
        // geometry_msgs::PoseStamped landingmsg;
        // landingmsg.header.stamp = ros::Time::now();
        // landingmsg.pose = home_pose_;
        // landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
        // target_pose_pub_.publish(landingmsg);
        // node_state = LANDED;
        // ros::spinOnce();
        // break;
    }
    case LANDED:
        ROS_INFO("Landed. Please set to position control and disarm.");
        cmdloop_timer_.stop();
        break;
    }
}

void dslsCtrl::statusloopCallback(const ros::TimerEvent &event) {
    if (sitl_enabled_) {
        // Enable OFFBoard mode and arm automatically
        // This will only run if the vehicle is simulated

        mavros_msgs::SetMode offb_set_mode_0, offb_set_mode_1;;
        offb_set_mode_0.request.custom_mode = "OFFBOARD";
        offb_set_mode_1.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd_0, arm_cmd_1;
        arm_cmd_0.request.value = true;
        arm_cmd_1.request.value = true;

        if( current_state_0_.mode != "OFFBOARD" && current_state_1_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(3.0))){
            if( set_mode_client_0_.call(offb_set_mode_0) && offb_set_mode_0.response.mode_sent && set_mode_client_1_.call(offb_set_mode_1) && offb_set_mode_1.response.mode_sent){
                ROS_INFO("[main] Offboard enabled");
            }
                last_request_ = ros::Time::now();
            } 

        else {
            if( !current_state_0_.armed && !current_state_1_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
                if( arming_client_0_.call(arm_cmd_0) && arming_client_1_.call(arm_cmd_1) && arm_cmd_0.response.success && arm_cmd_1.response.success) {
                    ROS_INFO("[main] Vehicle armed");
                }
                else {
                    ROS_INFO("[main] Arming Failure!");
                }
                last_request_ = ros::Time::now();
            }
        }
    }
    //   pubSystemStatus();
}

void dslsCtrl::pubDebugData(){
    dsls_state_pub_.publish(state18_);
    dea_state_pub_.publish(dea_xi4_);    
    dea_force_0_pub_.publish(dea_force_0_);
    dea_force_1_pub_.publish(dea_force_1_);    
    test_setpoint_0_pub_.publish(attitude_dea_0_);
    test_setpoint_1_pub_.publish(attitude_dea_1_);
    lpf_data_pub_.publish(lpf_data_);   
}


void dslsCtrl::dynamicReconfigureCallback(double_sls_controller::DoubleSLSControllerConfig &config, uint32_t level) {
    /* Mission */
    if(mission_enabled_ != config.mission_enabled) {
        if(mission_enabled_){
            loadDSLSDEARef(); // load ref when quitting mission mode
        }
        mission_enabled_ = config.mission_enabled;
        ROS_INFO("Reconfigure request : mission_enabled = %s ", mission_enabled_ ? "true" : "false");
    } 
    // Inner-Loop
    else if(throttle_offset_ != config.throttle_offset) {
        throttle_offset_ = config.throttle_offset;
        ROS_INFO("Reconfigure request : throttle_offset = %.2f ", config.throttle_offset);
    }    
    else if(att_ctrl_tau_ != config.att_ctrl_tau) {
        att_ctrl_tau_ = config.att_ctrl_tau;
        ROS_INFO("Reconfigure request : att_ctrl_tau_ = %.2f ", config.att_ctrl_tau);
    }        
    /* DEA */
    // Switch
    else if(dea_enabled_ != config.dea_enabled) {
        dea_enabled_ = config.dea_enabled;
        ROS_INFO("Reconfigure request : dea_enabled = %s ", config.dea_enabled?"true":"false");
    } 
    else if(dea_preintegrate_enabled_ != config.dea_preintegrate_enabled) {
        dea_preintegrate_enabled_ = config.dea_preintegrate_enabled;
        ROS_INFO("Reconfigure request : dea_preintegrate_enabled = %s ", config.dea_preintegrate_enabled?"true":"false");
    }
    // Reference
    else if(radium_scalar_ != config.radium_scalar) {
        radium_scalar_ = config.radium_scalar;
        dsls_dea_ref_[0] = r_1_ * radium_scalar_;
        dsls_dea_ref_[4] = r_2_ * radium_scalar_;
        dsls_dea_ref_[8] = r_3_ * radium_scalar_;
        ROS_INFO("Reconfigure request : radium_scalar_ = %.2f ", radium_scalar_);
    }
    else if(freq_scalar_ != config.freq_scalar) {
        freq_scalar_ = config.freq_scalar;
        dsls_dea_ref_[1] = fr_1_ * freq_scalar_;
        dsls_dea_ref_[5] = fr_2_ * freq_scalar_;
        dsls_dea_ref_[9] = fr_3_ * freq_scalar_;
        ROS_INFO("Reconfigure request : freq_scalar_ = %.2f ", freq_scalar_);
    }
    else if(r_1_ != config.radium_x) {
        r_1_ = config.radium_x;
        dsls_dea_ref_[0] = r_1_;
        ROS_INFO("Reconfigure request : radium_x = %.2f ", dsls_dea_ref_[0]);
    }
    else if(r_2_ != config.radium_y) {
        r_2_ = config.radium_y;
        dsls_dea_ref_[4] = r_2_;
        ROS_INFO("Reconfigure request : radium_y = %.2f ", dsls_dea_ref_[4]);
    }
    else if(r_3_ != config.radium_z) {
        r_3_ = config.radium_z;
        dsls_dea_ref_[8] = r_3_;
        ROS_INFO("Reconfigure request : radium_z = %.2f ", dsls_dea_ref_[8]);
    }
    else if(fr_1_ != config.freq_x) {
        fr_1_ = config.freq_x;
        dsls_dea_ref_[1] = fr_1_;
        ROS_INFO("Reconfigure request : freq_x = %.2f ", dsls_dea_ref_[1]);
    }
    else if(fr_2_ != config.freq_y) {
        fr_2_ = config.freq_y;
        dsls_dea_ref_[5] = fr_2_;
        ROS_INFO("Reconfigure request : freq_y = %.2f ", dsls_dea_ref_[5]);
    }
    else if(fr_3_ != config.freq_z) {
        fr_3_ = config.freq_z;
        dsls_dea_ref_[9] = fr_3_;
        ROS_INFO("Reconfigure request : freq_z = %.2f ", dsls_dea_ref_[9]);
    }
    else if(c_1_ != config.ref_x) {
        c_1_ = config.ref_x;
        dsls_dea_ref_[2] = c_1_;
        ROS_INFO("Reconfigure request : ref_x = %.2f ", dsls_dea_ref_[2]);
    }
    else if(c_2_ != config.ref_y) {
        c_2_ = config.ref_y;
        dsls_dea_ref_[6] = c_2_;
        ROS_INFO("Reconfigure request : ref_y = %.2f ", dsls_dea_ref_[6]);
    }
    else if(c_3_ != config.ref_z) {
        c_3_ = config.ref_z;
        dsls_dea_ref_[10] = c_3_;
        ROS_INFO("Reconfigure request : ref_z = %.2f ", dsls_dea_ref_[10]);
    }



    // Gains
    else if(dea_k_[0] != config.Kpos_x) {
        dea_k_[0] = config.Kpos_x;
        ROS_INFO("Reconfigure request : Kpos_x = %.2f ", dea_k_[0]);
    }
    else if(dea_k_[1] != config.Kpos_y) {
        dea_k_[1] = config.Kpos_y;
        ROS_INFO("Reconfigure request : Kpos_y = %.2f ", dea_k_[1]);
    }
    else if(dea_k_[2] != config.Kpos_z) {
        dea_k_[2] = config.Kpos_z;
        ROS_INFO("Reconfigure request : Kpos_z = %.2f ", dea_k_[2]);
    }
    else if(dea_k_[3] != config.Kq13_1) {
        dea_k_[3] = config.Kq13_1;
        ROS_INFO("Reconfigure request : Kq13_1 = %.2f ", dea_k_[3]);
    }
    else if(dea_k_[4] != config.Kq21_1) {
        dea_k_[4] = config.Kq21_1;
        ROS_INFO("Reconfigure request : Kq21_1 = %.2f ", dea_k_[4]);
    }
    else if(dea_k_[5] != config.Kq22_1) {
        dea_k_[5] = config.Kq22_1;
        ROS_INFO("Reconfigure request : Kq22_1 = %.2f ", dea_k_[5]);
    }



    else if(dea_k_[6] != config.Kvel_x) {
        dea_k_[6] = config.Kvel_x;
        ROS_INFO("Reconfigure request : Kvel_x = %.2f ", dea_k_[6]);
    }
    else if(dea_k_[7] != config.Kvel_y) {
        dea_k_[7] = config.Kvel_y;
        ROS_INFO("Reconfigure request : Kvel_y = %.2f ", dea_k_[7]);
    }
    else if(dea_k_[8] != config.Kvel_z) {
        dea_k_[8] = config.Kvel_z;
        ROS_INFO("Reconfigure request : Kvel_z = %.2f ", dea_k_[8]);
    }
    else if(dea_k_[9] != config.Kq13_2) {
        dea_k_[9] = config.Kq13_2;
        ROS_INFO("Reconfigure request : Kq13_2 = %.2f ", dea_k_[9]);
    }
    else if(dea_k_[10] != config.Kq21_2) {
        dea_k_[10] = config.Kq21_2;
        ROS_INFO("Reconfigure request : Kq21_2 = %.2f ", dea_k_[10]);
    }
    else if(dea_k_[11] != config.Kq22_2) {
        dea_k_[11] = config.Kq22_2;
        ROS_INFO("Reconfigure request : Kq22_2 = %.2f ", dea_k_[11]);
    }



    else if(dea_k_[12] != config.Kacc_x) {
        dea_k_[12] = config.Kacc_x;
        ROS_INFO("Reconfigure request : Kacc_x = %.2f ", dea_k_[12]);
    }
    else if(dea_k_[13] != config.Kacc_y) {
        dea_k_[13] = config.Kacc_y;
        ROS_INFO("Reconfigure request : Kacc_y = %.2f ", dea_k_[13]);
    }
    else if(dea_k_[14] != config.Kacc_z) {
        dea_k_[14] = config.Kacc_z;
        ROS_INFO("Reconfigure request : Kacc_z = %.2f ", dea_k_[14]);
    }



    else if(dea_k_[18] != config.Kjer_x) {
        dea_k_[18] = config.Kjer_x;
        ROS_INFO("Reconfigure request : Kjer_x = %.2f ", dea_k_[18]);
    }
    else if(dea_k_[19] != config.Kjer_y) {
        dea_k_[19] = config.Kjer_y;
        ROS_INFO("Reconfigure request : Kjer_y = %.2f ", dea_k_[19]);
    }
    else if(dea_k_[20] != config.Kjer_z) {
        dea_k_[20] = config.Kjer_z;
        ROS_INFO("Reconfigure request : Kjer_z = %.2f ", dea_k_[20]);
    }



}

gazebo_msgs::LinkState dslsCtrl::getLinkState(std::string link_name) {
    gazebo_msgs::LinkState current_state;
    gz_link_state_request_.request.link_name = string(link_name);
    gz_link_state_request_.request.reference_frame = string("world");
    if(gz_link_state_client_.call(gz_link_state_request_)){
        current_state = gz_link_state_request_.response.link_state;
    }
    return current_state;
}


geometry_msgs::Pose dslsCtrl::getLinkPose(std::string link_name){
    geometry_msgs::Pose current_pose;
    current_pose = this->getLinkState(link_name).pose;
    return current_pose;
}

geometry_msgs::Twist dslsCtrl::getLinkTwist(std::string link_name){
    geometry_msgs::Twist current_twist;
    current_twist = this->getLinkState(link_name).twist;
    return current_twist;
}

int dslsCtrl::executeMission(void) {
    if(!mission_enabled_ || !dea_enabled_) {
        if(mission_start_) mission_start_ = false;
        if(mission_stage_) mission_stage_ = 0; // Reset Mission Stage
        return 1;
    }

    if(!mission_start_) {
        saveDSLSDEARef();
        ROS_INFO_STREAM("[exeMission] Mission Start");
        mission_start_ = true;        
    }
    
    switch(mission_stage_) {
        case 0: // Set-point 0 (DEA Origin)
            if(!mission_ref_updated_){
                setDEASetpoint(dea_mission_setpoint0_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(20);
            break;
        case 1: // Set-point 1
            if(!mission_ref_updated_){
                setDEASetpoint(dea_mission_setpoint1_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(10);
            break;
        case 2: // Set-point 2
            if(!mission_ref_updated_){
                setDEASetpoint(dea_mission_setpoint2_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(10);
            break;
        case 3: // Set-point 
            if(!mission_ref_updated_){
                setDEASetpoint(dea_mission_setpoint3_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(10);
            break;
        case 4: // Back to Set-point 0 (DEA Origin)
            if(!mission_ref_updated_){
                setDEASetpoint(dea_mission_setpoint0_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(10);
            break;
        case 5: // Trajectory Tracking
            if(!mission_ref_updated_){
                setDEATrajectory(dea_mission_trajectory_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(40);
            break;
        case 6: // Back to Set-point 0 (DEA Origin)
            if(!mission_ref_updated_){
                setDEASetpoint(dea_mission_setpoint0_);
                mission_ref_updated_ = true;
            }
            checkMissionStage(10);
            break;            
        default: // Reset Mission
            if(ros::Time::now().toSec() - mission_time_last_ >= 10){
                ROS_INFO("[exeMission] Mission Accomplished");
                mission_time_last_ = ros::Time::now().toSec();
            }
    }
    return 0;
}

int dslsCtrl::checkMissionStage(double mission_time_span) {
    if(ros::Time::now().toSec() - mission_time_last_ >= mission_time_span) {
        mission_time_last_ = ros::Time::now().toSec();
        mission_stage_ += 1;
        mission_ref_updated_ = false;
        ROS_INFO_STREAM("[exeMission] Stage " << mission_stage_-1 << " ended, switching to stage " << mission_stage_);
        return 0;
    }
    return 1;
}

int dslsCtrl::setDEASetpoint(double dea_setpoint[6]) {
    dsls_dea_ref_[0] = 0.0;
    dsls_dea_ref_[4] = 0.0;
    dsls_dea_ref_[8] = 0.0;
    dsls_dea_ref_[2] = dea_setpoint[0];
    dsls_dea_ref_[6] = dea_setpoint[1];
    dsls_dea_ref_[10] = dea_setpoint[2];
    dsls_dea_ref_[12] = dea_setpoint[3];
    dsls_dea_ref_[13] = dea_setpoint[4];
    dsls_dea_ref_[14] = dea_setpoint[5];
    return 0;
}

int dslsCtrl::setDEATrajectory(double dea_trajectory[15]){
    for(int i = 0; i<15; i++){
        dsls_dea_ref_[i] = dea_trajectory[i];
    }
    return 0;
}

int dslsCtrl::saveDSLSDEARef(void) {
    for(int i=0; i<15; i++){
        dsls_dea_ref_temp_[i] = dsls_dea_ref_[i]; // save refs
    }
    return 0;
}

int dslsCtrl::loadDSLSDEARef(void) {
    for(int i=0; i<15; i++){
        dsls_dea_ref_[i] = dsls_dea_ref_temp_[i]; // save refs
    }
    return 0;
}

