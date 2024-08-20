#include <double_sls_controller/single_sls_controller.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>


/*============================== Helper Function Implementations ==============================*/


PendulumAngles ToPenAngles(double Lx,double Ly,double Lz) { //x=base.x
    PendulumAngles angles;
    double L = L_CABLE;

    /* beta (y-axis rotation) */ 
    double sinbeta = Lx/L;
    double cosbeta = Lz/(L*std::cos(angles.alpha));
    angles.beta = std::asin(sinbeta);

    /* alpha (x-axis rotation) */ 
    double cosa_cosb = Lz/L;
    double sina_cosb = Ly/-L;
    angles.alpha = std::asin(sina_cosb/std::cos(angles.beta));

    return angles;
}


void att_out_pub(ros::Publisher &att_con_pub, const double controller_output[3]){
    att_out.header.stamp = ros::Time::now();
    
    double roll,pitch,yaw, thrust;
    thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1]/thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);

    att_out.rpy[0] = roll;
    att_out.rpy[1] = pitch;
    att_out.rpy[2] = yaw;


    att_out.target_thrust = attitude.thrust;

    for (int i = 0; i < 3; i++){
        att_out.con_out[i] = controller_output[i];
    }

    att_con_pub.publish(att_out);
}


void pubPTState(ros::Publisher &sls_state_pub){
    PTState.header.stamp = ros::Time::now();
    PTState.PT_states[0] = current_local_pos.pose.position.x;
    PTState.PT_states[1] = -current_local_pos.pose.position.y;
    PTState.PT_states[2] = -current_local_pos.pose.position.z;
    PTState.PT_states[3] = sls_state1.alpha;
    PTState.PT_states[4] = sls_state1.beta;
    PTState.PT_states[5] = current_local_vel.twist.linear.x;
    PTState.PT_states[6] = -current_local_vel.twist.linear.y;
    PTState.PT_states[7] = -current_local_vel.twist.linear.z;
    PTState.PT_states[8] = sls_state1.gamma_alpha;
    PTState.PT_states[9] = sls_state1.gamma_beta;
    sls_state_pub.publish(PTState);
}


void force_attitude_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
    attitude.header.stamp = ros::Time::now();
    double roll, pitch, yaw, thrust;
    thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1]/thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);

    tf2::Quaternion attitudetarget_q;
    attitudetarget_q.setRPY(roll, pitch, yaw);
    attitude.orientation.x = attitudetarget_q.getX();
    attitude.orientation.y = attitudetarget_q.getY();
    attitude.orientation.z = attitudetarget_q.getZ();
    attitude.orientation.w = attitudetarget_q.getW();

    attitude.thrust = std::max(0.0, std::min(1.0, (thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
    attitude.type_mask = 1|2|4;
}


void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
    attitude.header.stamp = ros::Time::now();
    double roll, pitch, yaw, thrust;
    thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1]/thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);
    tf2::Quaternion attitudetarget_q;
    attitudetarget_q.setRPY(roll, pitch, yaw);
    attitude.orientation.x = attitudetarget_q.getX();
    attitude.orientation.y = attitudetarget_q.getY();
    attitude.orientation.z = attitudetarget_q.getZ();
    attitude.orientation.w = attitudetarget_q.getW();

    Eigen::Vector4d curr_att;
    Eigen::Vector4d ref_att;

    curr_att(0) = quadpose.orientation.w; //ROS_INFO_STREAM("quadpose.ori.w:" << quadpose.orientation.w);
    curr_att(1) = quadpose.orientation.x; //ROS_INFO_STREAM("quadpose.ori.x:" << quadpose.orientation.x);
    curr_att(2) = quadpose.orientation.y; //ROS_INFO_STREAM("quadpose.ori.y:" << quadpose.orientation.y);
    curr_att(3) = quadpose.orientation.z; //ROS_INFO_STREAM("quadpose.ori.z:" << quadpose.orientation.z);
    ref_att(0) = attitudetarget_q.getW();
    ref_att(1) = attitudetarget_q.getX();
    ref_att(2) = attitudetarget_q.getY();
    ref_att(3) = attitudetarget_q.getZ();

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

    if(quad_only_enabled){
        attitude.thrust = std::max(0.0, std::min(1.0, (thrust - thrust_0_q) / thrust_coeff + thrust_norm_hover));
    }

    else{
        attitude.thrust = std::max(0.0, std::min(1.0, (thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
    }

    attitude.type_mask = 128;
}


#if GAIN_MISMATCHED
void updateParams(const ros::NodeHandle &nh){
    nh.getParam("k_pos_z", K_POS_Z); Kv12[0] = K_VEL_Z;
    nh.getParam("k_pos_x", K_POS_X); Kv12[1] = K_JER_X;
    nh.getParam("k_pos_y", K_POS_Y); Kv12[2] = K_JER_Y; 
    nh.getParam("k_vel_z", K_VEL_Z); Kv12[3] = K_POS_Z;
    nh.getParam("k_vel_x", K_VEL_X); Kv12[4] = K_ACC_X;
    nh.getParam("k_vel_y", K_VEL_Y); Kv12[5] = K_ACC_Y;
    nh.getParam("k_acc_z", K_ACC_Z); Kv12[6] = 0;
    nh.getParam("k_acc_x", K_ACC_X); Kv12[7] = K_VEL_X;
    nh.getParam("k_acc_y", K_ACC_Y); Kv12[8] = K_VEL_Y;
    nh.getParam("k_jer_z", K_JER_Z); Kv12[9] = 0;
    nh.getParam("k_jer_x", K_JER_X); Kv12[10] = K_POS_X;
    nh.getParam("k_jer_y", K_JER_Y); Kv12[11] = K_POS_Y;
    nh.getParam("m_quad", M_QUAD); Param[0] = M_QUAD;
    nh.getParam("m_load", M_LOAD); Param[1] = M_LOAD;
    nh.getParam("l_cable", L_CABLE); Param[2] = L_CABLE;
    nh.getParam("gravity", GRAVITY); Param[3] = GRAVITY;
    nh.getParam("att_ctrl_tau", attctrl_tau_); 
}
#else
void updateParams(const ros::NodeHandle &nh){
    nh.getParam("k_pos_z", K_POS_Z); Kv12[0] = K_POS_Z;
    nh.getParam("k_pos_x", K_POS_X); Kv12[1] = K_POS_X;
    nh.getParam("k_pos_y", K_POS_Y); Kv12[2] = K_POS_Y; 
    nh.getParam("k_vel_z", K_VEL_Z); Kv12[3] = K_VEL_Z;
    nh.getParam("k_vel_x", K_VEL_X); Kv12[4] = K_VEL_X;
    nh.getParam("k_vel_y", K_VEL_Y); Kv12[5] = K_VEL_Y;
    nh.getParam("k_acc_z", K_ACC_Z); Kv12[6] = K_ACC_Z;
    nh.getParam("k_acc_x", K_ACC_X); Kv12[7] = K_ACC_X;
    nh.getParam("k_acc_y", K_ACC_Y); Kv12[8] = K_ACC_Y;
    nh.getParam("k_jer_z", K_JER_Z); Kv12[9] = K_JER_Z;
    nh.getParam("k_jer_x", K_JER_X); Kv12[10] = K_JER_X;
    nh.getParam("k_jer_y", K_JER_Y); Kv12[11] = K_JER_Y;
    nh.getParam("m_quad", M_QUAD); Param[0] = M_QUAD;
    nh.getParam("m_load", M_LOAD); Param[1] = M_LOAD;
    nh.getParam("l_cable", L_CABLE); Param[2] = L_CABLE;
    nh.getParam("gravity", GRAVITY); Param[3] = GRAVITY;
    nh.getParam("att_ctrl_tau", attctrl_tau_); 
}
#endif

void printParams(void){
    ROS_INFO_STREAM("K_POS_Z: " << K_POS_Z);
    ROS_INFO_STREAM("K_POS_X: " << K_POS_X);   
    ROS_INFO_STREAM("K_POS_Y: " << K_POS_Y);
    ROS_INFO_STREAM("K_VEL_Z: " << K_VEL_Z);
    ROS_INFO_STREAM("K_VEL_X: " << K_VEL_X);
    ROS_INFO_STREAM("K_VEL_Y: " << K_VEL_Y);
    ROS_INFO_STREAM("K_ACC_Z: " << K_ACC_Z);
    ROS_INFO_STREAM("K_ACC_X: " << K_ACC_X);
    ROS_INFO_STREAM("K_ACC_Y: " << K_ACC_Y);
    ROS_INFO_STREAM("K_JER_Z: " << K_JER_Z);
    ROS_INFO_STREAM("K_JER_X: " << K_JER_X);
    ROS_INFO_STREAM("K_JER_Y: " << K_JER_Y);

    for(int i = 0; i < 12; i++){
        ROS_INFO_STREAM("Kv[" << i << "]:" << Kv12[i]); 
    }

    for(int i = 0; i < 4; i++){
        ROS_INFO_STREAM("Param[" << i << "]:" << Param[i]); 
    }
    ROS_INFO_STREAM("att_ctrl_tau:" << attctrl_tau_);
}


void get_quad_states(void){
    sls_state1.x = quadpose.position.x;
    sls_state1.y = -quadpose.position.y;
    sls_state1.z = -quadpose.position.z;
    sls_state1.vx = quadtwist.linear.x;
    sls_state1.vy = -quadtwist.linear.y;
    sls_state1.vz = -quadtwist.linear.z;
}


void get_pend_states(void){
    double Lx = (loadpose.position.x) - (quadpose.position.x) ;
    double Ly = (-loadpose.position.y) - (-quadpose.position.y) ;
    double Lz = (-loadpose.position.z) - (-quadpose.position.z) ;
    penangle = ToPenAngles( Lx, Ly, -Lz ); // in the paper the definition of n3 are opposite to the Z axis of gazebo
    sls_state1.alpha = penangle.alpha;
    sls_state1.beta = penangle.beta;

    double L = L_CABLE;
    double g_alpha, g_beta;
    g_beta = ((loadtwist.linear.x) - (quadtwist.linear.x))/(L*std::cos(sls_state1.beta));
    g_alpha = ((-loadtwist.linear.y) - (-quadtwist.linear.y) - std::sin(sls_state1.alpha)*std::sin(sls_state1.beta)*g_beta*L)/(-std::cos(sls_state1.alpha)*std::cos(sls_state1.beta)*L);

    sls_state1.gamma_alpha = g_alpha;
    sls_state1.gamma_beta = g_beta;
}


void pubQuadControl(double Kv6[6], double setpoint[6]){
    // double controller_output[3];
    controller_output[0] = (-Kv6[0] * (sls_state1.x - setpoint[0]) - Kv6[3] * (sls_state1.vx - setpoint[3]))*M_QUAD;
    controller_output[1] = (-Kv6[1] * (sls_state1.y - setpoint[1]) - Kv6[4] * (sls_state1.vy - setpoint[4]))*M_QUAD;
    controller_output[2] = (-Kv6[2] * (sls_state1.z - setpoint[2]) - Kv6[5] * (sls_state1.vz - setpoint[5]) - 9.81)*M_QUAD;

    force_rate_convert(controller_output, attitude);        
}


/*============================== Callback Function Implementations ==============================*/


void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void posegetCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_local_pos = *msg;

    #if !SITL_ENABLED
    
    quadpose.position.x = current_local_pos.pose.position.x;
    quadpose.position.y = current_local_pos.pose.position.y;
    quadpose.position.z = current_local_pos.pose.position.z;

    quadtwist.linear.x = current_local_vel.twist.linear.x;
    quadtwist.linear.y = current_local_vel.twist.linear.y;
    quadtwist.linear.z = current_local_vel.twist.linear.z;

    quadpose.orientation.w = current_local_pos.pose.orientation.w;
    quadpose.orientation.x = current_local_pos.pose.orientation.x;
    quadpose.orientation.y = current_local_pos.pose.orientation.y;
    quadpose.orientation.z = current_local_pos.pose.orientation.z;

    loadpose.position.x = load_pose.pose.position.x;
    loadpose.position.y = load_pose.pose.position.y;
    loadpose.position.z = load_pose.pose.position.z;

    loadtwist.linear.x = load_vel.twist.linear.x;
    loadtwist.linear.y = load_vel.twist.linear.y;
    loadtwist.linear.z = load_vel.twist.linear.z;
    
    get_quad_states();

    get_pend_states();
    
    #endif
}


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_local_pos = *msg;
}


void velCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_local_vel = *msg;
}


void loadposeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    load_pose.header.frame_id = "map";
    
    load_pose.header.stamp = ros::Time::now();
    
    load_pose.pose.position.x = msg->transform.translation.x;
    load_pose.pose.position.y = msg->transform.translation.y;
    load_pose.pose.position.z = msg->transform.translation.z;
    load_pose.pose.orientation = msg->transform.rotation;
    
}

void quadposeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    load_pose.header.frame_id = "map";
    
    load_pose.header.stamp = ros::Time::now();
    
    load_pose.pose.position.x = msg->transform.translation.x;
    load_pose.pose.position.y = msg->transform.translation.y;
    load_pose.pose.position.z = msg->transform.translation.z;
    load_pose.pose.orientation = msg->transform.rotation;
    
}


void attitudetargetCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    attitude = *msg;
}


void slsstateCallback(const double_sls_controller::PTStates::ConstPtr& msg){
    PTState = *msg;
}


void timerCallback(const ros::TimerEvent&){
    load_vel.header.frame_id = "map";
    load_vel.header.stamp = ros::Time::now();
    load_vel.twist.linear.x = (load_pose.pose.position.x - load_pose0.pose.position.x)/diff_time;
    load_vel.twist.linear.y = (load_pose.pose.position.y - load_pose0.pose.position.y)/diff_time;
    load_vel.twist.linear.z = (load_pose.pose.position.z - load_pose0.pose.position.z)/diff_time;

    load_pose0.pose.position.x = load_pose.pose.position.x;
    load_pose0.pose.position.y = load_pose.pose.position.y;
    load_pose0.pose.position.z = load_pose.pose.position.z;
}

#if SITL_ENABLED
void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){

    quadpose = msg->pose[2];
    quadtwist = msg->twist[2];    
    pendpose = msg->pose[9];    // 9: pendulum
    pendtwist = msg->twist[9];
    loadpose = msg->pose[10];   // 10: load; 
    loadtwist = msg->twist[10]; 

    tf2::Quaternion quad_q(quadpose.orientation.x, quadpose.orientation.y, quadpose.orientation.z, quadpose.orientation.w);
    tf2::Matrix3x3 quad_m(quad_q);
    double quad_roll, quad_pitch, quad_yaw;
    quad_m.getRPY(quad_roll, quad_pitch, quad_yaw);

    get_quad_states();
    get_pend_states();

    updateLoadPose(msg);
    updateQuadPose(msg);
    updateLoadVel();
    updateQuadVel();
    updatePendAngle();
    updateSingleSlsState();

    // diff_time = (ros::Time::now().toSec() - gazebo_last_called);
    // gazebo_last_called = ros::Time::now().toSec();

    for(int i=0; i<10;i++){
        ROS_INFO_STREAM(single_sls_state[i]);
    }
}

void gazeboSingleSlsCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){

    // quadpose = msg->pose[2];
    // quadtwist = msg->twist[2];    
    // pendpose = msg->pose[9];    // 9: pendulum
    // pendtwist = msg->twist[9];
    // loadpose = msg->pose[10];   // 10: load; 
    // loadtwist = msg->twist[10]; 

    // tf2::Quaternion quad_q(quadpose.orientation.x, quadpose.orientation.y, quadpose.orientation.z, quadpose.orientation.w);
    // tf2::Matrix3x3 quad_m(quad_q);
    // double quad_roll, quad_pitch, quad_yaw;
    // quad_m.getRPY(quad_roll, quad_pitch, quad_yaw);

    get_quad_states();
    get_pend_states();

    updateLoadPose(msg);
    updateQuadPose(msg);
    updateLoadVel();
    updateQuadVel();
    updatePendAngle();
    updateSingleSlsState();

    // diff_time = (ros::Time::now().toSec() - gazebo_last_called);
    // gazebo_last_called = ros::Time::now().toSec();

    // for(int i=0; i<10;i++){
    //     ROS_INFO_STREAM(single_sls_state[i]);
    // }
}

#endif

// New Helper Functions

void updateLoadPose(const gazebo_msgs::LinkStates::ConstPtr& msg){ // Update Load Position (states 1, 2, 3)
    load_1_pose.pose.position.x = msg->pose[10].position.x;
    load_1_pose.pose.position.y = msg->pose[10].position.y;
    load_1_pose.pose.position.z = msg->pose[10].position.z;
}

void updateQuadPose(const gazebo_msgs::LinkStates::ConstPtr& msg){
    quad_1_pose.pose.position.x = msg->pose[2].position.x;
    quad_1_pose.pose.position.y = msg->pose[2].position.y;
    quad_1_pose.pose.position.z = msg->pose[2].position.z;
}

void updateLoadVel(void){ // Update Load Velocity (states 7, 8, 9)
    load_1_vel.header.frame_id = "map";
    load_1_vel.header.stamp = ros::Time::now();
    load_1_vel.twist.linear.x = (load_1_pose.pose.position.x - load_1_pose_last.pose.position.x)/diff_time;
    load_1_vel.twist.linear.y = (load_1_pose.pose.position.y - load_1_pose_last.pose.position.y)/diff_time;
    load_1_vel.twist.linear.z = (load_1_pose.pose.position.z - load_1_pose_last.pose.position.z)/diff_time;
    load_1_pose_last.pose.position.x = load_1_pose.pose.position.x;
    load_1_pose_last.pose.position.y = load_1_pose.pose.position.y;
    load_1_pose_last.pose.position.z = load_1_pose.pose.position.z;
}

void updateQuadVel(void){
    quad_1_vel.header.frame_id = "map";
    quad_1_vel.header.stamp = ros::Time::now();
    quad_1_vel.twist.linear.x = (quad_1_pose.pose.position.x - quad_1_pose_last.pose.position.x)/diff_time;
    quad_1_vel.twist.linear.y = (quad_1_pose.pose.position.y - quad_1_pose_last.pose.position.y)/diff_time;
    quad_1_vel.twist.linear.z = (quad_1_pose.pose.position.z - quad_1_pose_last.pose.position.z)/diff_time;
    quad_1_pose_last.pose.position.x = quad_1_pose.pose.position.x;
    quad_1_pose_last.pose.position.y = quad_1_pose.pose.position.y;
    quad_1_pose_last.pose.position.z = quad_1_pose.pose.position.z;
}

void updatePendAngle(void){ // Update Pendulum Angle & Rate(states 4, 5, 9, 10)
    double Lx = (load_1_pose.pose.position.x) - (quad_1_pose.pose.position.x) ;
    double Ly = (-load_1_pose.pose.position.y) - (-quad_1_pose.pose.position.y) ;
    double Lz = (-load_1_pose.pose.position.z) - (-quad_1_pose.pose.position.z) ;
    double L = L_CABLE;
    
    /* beta (y-axis rotation) */ 
    double sinbeta = Lx/L;
    double cosbeta = Lz/(L*std::cos(pend_1_angle.alpha));
    /* alpha (x-axis rotation) */ 
    double cosa_cosb = Lz/L;
    double sina_cosb = Ly/-L;
    pend_1_angle.alpha = std::asin(sina_cosb/std::cos(pend_1_angle.beta));
    pend_1_angle.beta = std::asin(sinbeta);

    double g_alpha, g_beta;
    g_beta = ((load_1_vel.twist.linear.x) - (quad_1_vel.twist.linear.x))/(L*std::cos(pend_1_angle.beta));
    g_alpha = ((-load_1_vel.twist.linear.y) - (-quad_1_vel.twist.linear.y) - std::sin(pend_1_angle.alpha)*std::sin(pend_1_angle.beta)*g_beta*L)/(-std::cos(pend_1_angle.alpha)*std::cos(pend_1_angle.beta)*L);
    pend_1_angle.g_alpha = g_alpha;
    pend_1_angle.g_beta = g_beta;
}

void updateSingleSlsState(void){
    single_sls_state[0]    =   load_1_pose.pose.position.x;
    single_sls_state[1]    =   -load_1_pose.pose.position.y;
    single_sls_state[2]    =   -load_1_pose.pose.position.z;
    single_sls_state[3]    =   pend_1_angle.alpha;
    single_sls_state[4]    =   pend_1_angle.beta;
    single_sls_state[5]    =   load_1_vel.twist.linear.x;
    single_sls_state[6]    =   -load_1_vel.twist.linear.y;
    single_sls_state[7]    =   -load_1_vel.twist.linear.z;    
    single_sls_state[8]    =   pend_1_angle.g_alpha;
    single_sls_state[9]    =   pend_1_angle.g_beta;
}

void updateSingleSlsTarget(double controller_output[3], mavros_msgs::AttitudeTarget &attitude){
    attitude.header.stamp = ros::Time::now();
    double roll, pitch, yaw, thrust;
    thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
    yaw = 0;
    roll = std::asin(controller_output[1]/thrust);
    pitch = std::atan2(controller_output[0], -controller_output[2]);
    tf2::Quaternion attitudetarget_q;
    // attitudetarget_q.setRPY(roll, pitch, yaw);
    // attitude.orientation.x = attitudetarget_q.getX();
    // attitude.orientation.y = attitudetarget_q.getY();
    // attitude.orientation.z = attitudetarget_q.getZ();
    // attitude.orientation.w = attitudetarget_q.getW();

    Eigen::Vector4d curr_att;
    Eigen::Vector4d ref_att;

    curr_att(0) = quad_1_pose.pose.orientation.w; ROS_INFO_STREAM("quadpose.ori.w:" << quad_1_pose.pose.orientation.w);
    curr_att(1) = quad_1_pose.pose.orientation.x; ROS_INFO_STREAM("quadpose.ori.x:" << quad_1_pose.pose.orientation.x);
    curr_att(2) = quad_1_pose.pose.orientation.y; ROS_INFO_STREAM("quadpose.ori.y:" << quad_1_pose.pose.orientation.y);
    curr_att(3) = quad_1_pose.pose.orientation.z; ROS_INFO_STREAM("quadpose.ori.z:" << quad_1_pose.pose.orientation.z);
    ref_att(0) = attitudetarget_q.getW();
    ref_att(1) = attitudetarget_q.getX();
    ref_att(2) = attitudetarget_q.getY();
    ref_att(3) = attitudetarget_q.getZ();

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); // ROS_INFO_STREAM("qe:" << qe);
    Eigen::Vector3d desired_rate_{Eigen::Vector3d::Zero()};
    desired_rate_(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    desired_rate_(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    desired_rate_(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);   

    attitude.body_rate.x = desired_rate_(0); ROS_INFO_STREAM("body_rate_x:" << attitude.body_rate.x << " desired_rate_(0):" << desired_rate_(0));
    attitude.body_rate.y = desired_rate_(1); ROS_INFO_STREAM("body_rate_y:" << attitude.body_rate.y << " desired_rate_(1):" << desired_rate_(1));
    attitude.body_rate.z = desired_rate_(2); ROS_INFO_STREAM("body_rate_z:" << attitude.body_rate.z << " desired_rate_(2):" << desired_rate_(2));
    attitude.thrust = std::max(0.0, std::min(1.0, (thrust - thrust_0) / thrust_coeff + thrust_norm_hover));

    attitude.type_mask = 128;
}

void pubSingleSlsTarget(ros::Publisher &attitude_target_pub,mavros_msgs::AttitudeTarget & attitude){
    attitude.header.stamp = ros::Time::now();
    attitude_target_pub.publish(attitude);
}