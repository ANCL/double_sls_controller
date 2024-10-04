// /**
//  * @file offb_node.cpp
//  * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
//  * Stack and tested in Gazebo SITL
//  */

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Vector3Stamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>
// #include <dynamic_reconfigure/server.h>
// #include <double_sls_controller/configConfig.h>
// #include <double_sls_controller/double_sls_controller.h>
// #include <double_sls_controller/common.h>
// #include <double_sls_controller/control.h>
// #include <double_sls_controller/DSlsState.h>
// #include <double_sls_controller/DEAState.h>
// #include <double_sls_controller/LPFData.h>
// #include <DEAController.h>
// #include <DSLSDEAController.h>
// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <string>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #define MIN_DELTA_TIME 1e-16
// #define PI 3.1415926535


// /* External Data Source File Name */
// std::string f1_filename = "F1.csv";
// std::string f2_filename = "F2.csv";

// /* Messages */
// mavros_msgs::State current_state_0;
// mavros_msgs::State current_state_1;
// mavros_msgs::AttitudeTarget attitude_0;
// mavros_msgs::AttitudeTarget attitude_1;
// mavros_msgs::AttitudeTarget attitude_dea_0;
// mavros_msgs::AttitudeTarget attitude_dea_1;
// double_sls_controller::DSlsState state18;
// double_sls_controller::DEAState dea_xi4;
// double_sls_controller::LPFData lpf_data;

// void stateCb_0(const mavros_msgs::State::ConstPtr& msg);
// void stateCb_1(const mavros_msgs::State::ConstPtr& msg);
// void gazeboCb(const gazebo_msgs::LinkStates::ConstPtr& msg, ros::Publisher* attitude_setpoint_pub_0, ros::Publisher* attitude_setpoint_pub_1);
// void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav);
// void applyQuad0Controller(double Kv6[6], double setpoint[6]);
// void applyQuad1Controller(double Kv6[6], double setpoint[6]);
// int applyOpenLoopController(void);
// void applyDEAController(double_sls_controller::DSlsState state18, double_sls_controller::DEAState &dea_xi4, const double dea_k[24], const double dea_param[4], const double ref[13], double t);
// int applyDSLSDEAController(double_sls_controller::DSlsState state18, double_sls_controller::DEAState &dea_xi4, const double dea_k[24], const double dea_param[4], const double ref[15], double t);
// void getTargetAttitude(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav);
// geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, const geometry_msgs::Vector3 v2);

// double applyLPF(double x, double x_last, double x_last_2, double x_dot_last, double x_dot_last_2, double diff_time, double tau, double xi, double omega, int order);
// geometry_msgs::Vector3 applyLPFVector3(
//     geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, geometry_msgs::Vector3 v_last_2, geometry_msgs::Vector3 v_dot_last, geometry_msgs::Vector3 v_dot_last_2, 
//     double diff_time, double tau, double xi, double omega, int order
//     );

// double applyFiniteDiff(double x, double x_last, double diff_time);
// geometry_msgs::Vector3 applyFiniteDiffVector3(geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, double diff_time);
// int enu2ned(void);
// int enu2esd(void);

// /* booleans */
// bool dea_enabled = false;
// bool dea_last_status = false;
// bool sitl_enabled = true;
// bool param_tuning_enabled = false;
// bool dynamic_reconfigure_enabled = false;
// bool use_ned_frame = false;
// bool open_loop_ctrl_enabled = false;
// bool gazebo_omega_enabled = false;
// bool att_saturation_enabled = true;


// /* physical parameters */
// double max_fb_acc = 10.0;
// double uav_mass = 1.55;
// double load_mass = 0.25;
// double cable_length = 0.85;
// double gravity_acc = 9.81;
// const double dea_param[4] = {load_mass, uav_mass, cable_length, gravity_acc};

// /* Reference Trajectory & Pend Angle*/
// double trajectory_tracking_flag = 0;
// const double r_1 = 5.0 * trajectory_tracking_flag;
// const double fr_1 = 1.0;
// const double c_1 = -3;
// const double ph_1 = 0;
// const double r_2 = 5.0 * trajectory_tracking_flag;
// const double fr_2 = 1.0;
// const double c_2 = -2;
// const double ph_2 = PI/2;
// const double r_3 = 0 * trajectory_tracking_flag;
// const double fr_3 = 0.0;
// const double c_3 = -1.836683728556403;
// const double ph_3 = 0;
// double pend_angle_deg = 90;
// const double q_1_3_r = 0.7406322196911044;
// const double q_2_1_r = 0;
// const double q_2_2_r = -0.6717825272800765;
// const double dea_ref[13] = {r_1, fr_1, c_1, ph_1, r_2, fr_2, c_2, ph_2, r_3, fr_3, c_3, ph_3, pend_angle_deg};
// const double dsls_dea_ref[15] = {r_1, fr_1, c_1, ph_1, r_2, fr_2, c_2, ph_2, r_3, fr_3, c_3, ph_3, q_1_3_r, q_2_1_r, q_2_2_r};

// /* lpf parameters*/
// const double lpf_tau = 0.01;
// const double lpf_xi = 0.7;
// const double lpf_omega = 100;

// /* (Primed) Initial Conditions */
// double load_pose_ic[3] = {0, 0, c_3};
// double setpoint_0[6] = {load_pose_ic[0], load_pose_ic[1] - cable_length*sin(pend_angle_deg * 0.5),  load_pose_ic[2] - cable_length*cos(pend_angle_deg * 0.5), 0, 0, 0};
// double setpoint_1[6] = {load_pose_ic[0], load_pose_ic[1] + cable_length*sin(pend_angle_deg * 0.5),  load_pose_ic[2] - cable_length*cos(pend_angle_deg * 0.5), 0, 0, 0};
// // double setpoint_0[6] = {load_pose_ic[0] + cable_length*sin(pend_angle_deg * 0.5), load_pose_ic[1], load_pose_ic[2] - cable_length*cos(pend_angle_deg * 0.5), 0, 0, 0};
// // double setpoint_1[6] = {load_pose_ic[0] - cable_length*sin(pend_angle_deg * 0.5), load_pose_ic[1], load_pose_ic[2] - cable_length*cos(pend_angle_deg * 0.5), 0, 0, 0};
// double dea_xi4_ic[4] = {-gravity_acc, -5.10, 0, 0}; 

// /* Gains */
// double Kv6[6] = {4.3166, 4.3166, 4.316, 3.1037, 3.1037, 3.1037};

// /* Time */
// double gazebo_last_called;
// double controller_last_called;
// double dea_start_time; 
// double dea_end_time; 

// /* Gazebo Index Matching */
// bool gazebo_link_name_matched = false;
// int uav0_link_index;
// int uav1_link_index;
// int pend0_link_index;
// int pend1_link_index;
// int load_link_index;
// const char* link_name[5] = {
//     "px4vision_0::base_link", 
//     "px4vision_1::base_link",
//     "slung_load::pendulum_0::base_link",
//     "slung_load::pendulum_1::base_link",
//     "slung_load::load::base_link"
//     };

// /* uav0 */
// geometry_msgs::PoseStamped uav0_pose, uav0_pose_last;
// geometry_msgs::TwistStamped uav0_twist, uav0_twist_last;
// /* uav1 */
// geometry_msgs::PoseStamped uav1_pose, uav1_pose_last;
// geometry_msgs::TwistStamped uav1_twist, uav1_twist_last;
// /* pend0 */
// geometry_msgs::Vector3 pend0_q, pend0_q_last, pend0_q_last_2;
// geometry_msgs::Vector3 pend0_q_dot;
// geometry_msgs::Vector3 pend0_q_dot_lpf1, pend0_q_dot_lpf1_last, pend0_q_dot_lpf1_last_2;
// geometry_msgs::Vector3 pend0_q_dot_lpf2, pend0_q_dot_lpf2_last, pend0_q_dot_lpf2_last_2;
// geometry_msgs::Vector3 pend0_omega;
// /* pend1 */
// geometry_msgs::Vector3 pend1_q, pend1_q_last, pend1_q_last_2;
// geometry_msgs::Vector3 pend1_q_dot;
// geometry_msgs::Vector3 pend1_q_dot_lpf1, pend1_q_dot_lpf1_last, pend1_q_dot_lpf1_last_2;
// geometry_msgs::Vector3 pend1_q_dot_lpf2, pend1_q_dot_lpf2_last, pend1_q_dot_lpf2_last_2;
// geometry_msgs::Vector3 pend1_omega;
// /* load */
// geometry_msgs::PoseStamped load_pose, load_pose_last;
// geometry_msgs::TwistStamped load_twist;
// /* DEA Controller Output Force */
// geometry_msgs::Vector3Stamped dea_force_0;
// geometry_msgs::Vector3Stamped dea_force_1;


// int pubDebugData(
//     ros::Publisher &dsls_state_pub, double_sls_controller::DSlsState &state18,
//     ros::Publisher &dea_state_pub, double_sls_controller::DEAState &dea_xi4,   
//     ros::Publisher &dea_force_0_pub, geometry_msgs::Vector3Stamped &dea_force_0, 
//     ros::Publisher &dea_force_1_pub, geometry_msgs::Vector3Stamped &dea_force_1, 
//     ros::Publisher &test_setpoint_0_pub, mavros_msgs::AttitudeTarget &attitude_dea_0, 
//     ros::Publisher &test_setpoint_1_pub, mavros_msgs::AttitudeTarget &attitude_dea_1,
//     ros::Publisher &lpf_data_pub, double_sls_controller::LPFData &lpf_data
// ){
//     dsls_state_pub.publish(state18);
//     dea_state_pub.publish(dea_xi4);    
//     dea_force_0_pub.publish(dea_force_0);
//     dea_force_1_pub.publish(dea_force_1);    
//     test_setpoint_0_pub.publish(attitude_dea_0);
//     test_setpoint_1_pub.publish(attitude_dea_1);
//     lpf_data_pub.publish(lpf_data);
//     return 0;
// }


// int main(int argc, char **argv){

//     /* ROS Node Utilities */
//     ros::init(argc, argv, "double_sls_node");
//     ros::NodeHandle nh("");
//     ros::NodeHandle nh_private("~");
//     ros::Subscriber state_sub_0 = nh.subscribe<mavros_msgs::State> ("/uav0/mavros/state", 10, stateCb_0);  
//     ros::Subscriber state_sub_1 = nh.subscribe<mavros_msgs::State> ("/uav1/mavros/state", 10, stateCb_1);   
//     ros::Publisher local_pos_pub_0 = nh.advertise<geometry_msgs::PoseStamped> ("/uav0/mavros/setpoint_position/local", 10);
//     ros::Publisher local_pos_pub_1 = nh.advertise<geometry_msgs::PoseStamped> ("/uav1/mavros/setpoint_position/local", 10);
//     ros::Publisher attitude_setpoint_pub_0 = nh.advertise<mavros_msgs::AttitudeTarget> ("/uav0/mavros/setpoint_raw/attitude", 10);
//     ros::Publisher attitude_setpoint_pub_1 = nh.advertise<mavros_msgs::AttitudeTarget> ("/uav1/mavros/setpoint_raw/attitude", 10);
//     ros::Publisher test_setpoint_0_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/double_sls_controller/setpoint_raw/attitude_0", 10);
//     ros::Publisher test_setpoint_1_pub = nh.advertise<mavros_msgs::AttitudeTarget> ("/double_sls_controller/setpoint_raw/attitude_1", 10);
//     ros::Publisher dea_force_0_pub = nh.advertise<geometry_msgs::Vector3Stamped> ("/double_sls_controller/dea_force_0", 10);
//     ros::Publisher dea_force_1_pub = nh.advertise<geometry_msgs::Vector3Stamped> ("/double_sls_controller/dea_force_1", 10);
//     ros::Publisher dsls_state_pub = nh.advertise<double_sls_controller::DSlsState> ("/double_sls_controller/dsls_state", 10);
//     ros::Publisher lpf_data_pub = nh.advertise<double_sls_controller::LPFData> ("/double_sls_controller/lpf_data", 10);
//     ros::Publisher dea_state_pub = nh.advertise<double_sls_controller::DEAState> ("/double_sls_controller/dea_state", 10);    
//     ros::ServiceClient arming_client_0 = nh.serviceClient<mavros_msgs::CommandBool> ("/uav0/mavros/cmd/arming");
//     ros::ServiceClient arming_client_1 = nh.serviceClient<mavros_msgs::CommandBool> ("/uav1/mavros/cmd/arming");
//     ros::ServiceClient set_mode_client_0 = nh.serviceClient<mavros_msgs::SetMode> ("/uav0/mavros/set_mode");
//     ros::ServiceClient set_mode_client_1 = nh.serviceClient<mavros_msgs::SetMode> ("/uav1/mavros/set_mode");
//     ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, boost::bind(gazeboCb, _1, &attitude_setpoint_pub_0, &attitude_setpoint_pub_1));
//     // ros::Subscriber gazebo_pose_sub = nh.subscribe<gazebo_msgs::PoseStamped>("/gazebo/default/pose/info", 10, gazeboPoseCb);
//     // ros::Timer cmdloop_timer = nh.createTimer(ros::Duration(0.01), cmdloopCallback);
//     // ros::Timer statusloop_timer = nh.createTimer(ros::Duration(1), statusloopCallback);
//     ros::Rate rate(50.0); //the setpoint publishing rate MUST be faster than 2Hz
    

//     if(!dynamic_reconfigure_enabled){
//         dynamic_reconfigure::Server<double_sls_controller::configConfig> server;
//         dynamic_reconfigure::Server<double_sls_controller::configConfig>::CallbackType f;
//         f = boost::bind(&callback, _1, _2);
//         server.setCallback(f);
//     }

//     const double dea_k1[4] = {0.4025,    2.1325,    4.0800,   3.3500};
//     const double dea_k2[4] = {0.4025,    2.1325,    4.0800,   3.3500};
//     const double dea_k3[4] = {24.0000,   50.0000,   35.0000,   10.0000};
//     const double dea_k4[4] = {2.0000,    3.0000,         0,         0};
//     const double dea_k5[4] = {2.0000,    3.0000,         0,         0};
//     const double dea_k6[4] = {2.0000,    3.0000,         0,         0};
    
//     if(!param_tuning_enabled){
//         for(int i = 0; i < 4; i++){
//             dea_k[0 + i*6] = dea_k1[i];
//             dea_k[1 + i*6] = dea_k2[i];
//             dea_k[2 + i*6] = dea_k3[i];
//             dea_k[3 + i*6] = dea_k4[i];
//             dea_k[4 + i*6] = dea_k5[i];
//             dea_k[5 + i*6] = dea_k6[i];
//         }
//     }

//     for(int j=0; j<4; j++) dea_xi4.dea_xi4[j] = dea_xi4_ic[j];      
 
//     // wait for FCU connection
//     while(ros::ok() && !current_state_0.connected && !current_state_1.connected){
//         ros::spinOnce();
//         rate.sleep();
//     }

//     geometry_msgs::PoseStamped pose_0;
//     pose_0.pose.position.x = 0;
//     pose_0.pose.position.y = 0.8;
//     pose_0.pose.position.z = 2.0;

//     geometry_msgs::PoseStamped pose_1;
//     pose_1.pose.position.x = 0;
//     pose_1.pose.position.y = -0.8;
//     pose_1.pose.position.z = 2.0;

//     //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub_0.publish(pose_0);
//         local_pos_pub_1.publish(pose_1);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     mavros_msgs::SetMode offb_set_mode_0, offb_set_mode_1;;
//     offb_set_mode_0.request.custom_mode = "OFFBOARD";
//     offb_set_mode_1.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd_0, arm_cmd_1;
//     arm_cmd_0.request.value = true;
//     arm_cmd_1.request.value = true;

//     ros::Time last_request = ros::Time::now();
//     ros::Time node_start_time = ros::Time::now();

//     nh.getParam("bool", dea_enabled);
//     dea_last_status = dea_enabled;

//     while(ros::ok()){
//         nh.getParam("bool", dea_enabled);
//         if(dea_enabled && dea_enabled != dea_last_status) {
//             dea_start_time = ros::Time::now().toSec();
//             ROS_INFO_STREAM("[main] DEA controller enabled");
//         }
//         else if(!dea_enabled && dea_enabled != dea_last_status){
//             dea_end_time = ros::Time::now().toSec();
//             ROS_INFO_STREAM("[main] DEA controller disabled");
//         } 
//         dea_last_status = dea_enabled;

//         if(param_tuning_enabled){
//             std::vector<double> dea_k_vec;
//             nh.getParam("dea_k24", dea_k_vec);
//             for(size_t i=0; i < 24; ++i) dea_k[i] = dea_k_vec[i];
//         }

//         if( current_state_0.mode != "OFFBOARD" && current_state_1.mode != "OFFBOARD" &&
//             (ros::Time::now() - last_request > ros::Duration(3.0))){
//             if( set_mode_client_0.call(offb_set_mode_0) && offb_set_mode_0.response.mode_sent &&
//                 set_mode_client_1.call(offb_set_mode_1) && offb_set_mode_1.response.mode_sent
//             ){
//                 ROS_INFO("[main] Offboard enabled");
//             }
//             last_request = ros::Time::now();
//         } 
        
//         else {
//             if( !current_state_0.armed && !current_state_1.armed &&
//                 (ros::Time::now() - last_request > ros::Duration(5.0))){
//                 if( arming_client_0.call(arm_cmd_0) && arming_client_1.call(arm_cmd_1) && arm_cmd_0.response.success && arm_cmd_1.response.success){
//                     ROS_INFO("[main] Vehicle armed");
//                 }
//                 else{
//                     ROS_INFO("[main] Arming Failure!");
//                 }
//                 last_request = ros::Time::now();
//             }
//         }

//         // ROS_INFO_STREAM("uav1_pose.pose.ori.w:" << uav1_pose.pose.orientation.w);    
//         // ROS_INFO_STREAM("uav1_pose.pose.ori.x:" << uav1_pose.pose.orientation.x);
//         // ROS_INFO_STREAM("uav1_pose.pose.ori.y:" << uav1_pose.pose.orientation.y);
//         // ROS_INFO_STREAM("uav1_pose.pose.ori.z:" << uav1_pose.pose.orientation.z);  
        

//         pubDebugData(
//             dsls_state_pub, state18, 
//             dea_state_pub, dea_xi4,
//             dea_force_0_pub, dea_force_0, 
//             dea_force_1_pub, dea_force_1,            
//             test_setpoint_0_pub, attitude_dea_0, 
//             test_setpoint_1_pub, attitude_dea_1,
//             lpf_data_pub, lpf_data
//         );

//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }


// void stateCb_0(const mavros_msgs::State::ConstPtr& msg){
//     current_state_0 = *msg;
// }

// void stateCb_1(const mavros_msgs::State::ConstPtr& msg){
//     current_state_1 = *msg;
// }

// void gazeboCb(const gazebo_msgs::LinkStates::ConstPtr& msg, ros::Publisher* attitude_setpoint_pub_0, ros::Publisher* attitude_setpoint_pub_1){
//     /* Match links on the first call*/
//     if(!gazebo_link_name_matched){
//         ROS_INFO("[gazeboCb] Matching Gazebo Links");
//         int temp_index[5];
//         for(int i=0; i<23; i++){
//             for(int j=0; j<5; j++){
//                 if(msg->name[i] == link_name[j]){
//                     temp_index[j] = i;
//                 };
//             }
//         }
//         uav0_link_index = temp_index[0];    ROS_INFO_STREAM("[gazeboCb] uav0_link_index=" << uav0_link_index);
//         uav1_link_index = temp_index[1];    ROS_INFO_STREAM("[gazeboCb] uav1_link_index=" << uav1_link_index);
//         pend0_link_index = temp_index[2];   ROS_INFO_STREAM("[gazeboCb] pend0_link_index=" << pend0_link_index);
//         pend1_link_index = temp_index[3];   ROS_INFO_STREAM("[gazeboCb] pend0_link_index=" << pend1_link_index);
//         load_link_index = temp_index[4];    ROS_INFO_STREAM("[gazeboCb] load_link_index=" << load_link_index);
//         gazebo_link_name_matched = true;
//         ROS_INFO("[gazeboCb] Matching Complete");
//     }

//     double diff_time;
//     diff_time = (ros::Time::now().toSec() - gazebo_last_called); 
//     gazebo_last_called = ros::Time::now().toSec();
//     printf("[gazeboCb] diff_time = %.10f\n", diff_time);
//     uav0_pose.pose = msg -> pose[uav0_link_index];
//     uav0_twist.twist = msg -> twist[uav0_link_index];
//     uav1_pose.pose = msg -> pose[uav1_link_index];
//     uav1_twist.twist = msg -> twist[uav1_link_index];
//     load_pose.pose = msg -> pose[load_link_index];
//     load_twist.twist = msg -> twist[load_link_index];

//     // coordinate transform
//     if(use_ned_frame) enu2ned();
//     else enu2esd();
    
//     // q1
//     pend0_q.x = (load_pose.pose.position.x - uav0_pose.pose.position.x);
//     pend0_q.y = (load_pose.pose.position.y - uav0_pose.pose.position.y);
//     pend0_q.z = (load_pose.pose.position.z - uav0_pose.pose.position.z);
//     double q0_norm = sqrt(pend0_q.x*pend0_q.x + pend0_q.y*pend0_q.y + pend0_q.z*pend0_q.z);
//     pend0_q.x = pend0_q.x / q0_norm;
//     pend0_q.y = pend0_q.y / q0_norm;
//     pend0_q.z = pend0_q.z / q0_norm;
//     // q2
//     pend1_q.x = (load_pose.pose.position.x - uav1_pose.pose.position.x);
//     pend1_q.y = (load_pose.pose.position.y - uav1_pose.pose.position.y);
//     pend1_q.z = (load_pose.pose.position.z - uav1_pose.pose.position.z);
//     double q1_norm = sqrt(pend1_q.x*pend1_q.x + pend1_q.y*pend1_q.y + pend1_q.z*pend1_q.z);
//     pend1_q.x = pend1_q.x / q1_norm;
//     pend1_q.y = pend1_q.y / q1_norm;
//     pend1_q.z = pend1_q.z / q1_norm;

//     if(diff_time > MIN_DELTA_TIME){    
//         // pend0_q_dot.x = (pend0_q.x - pend0_q_last.x) / diff_time;
//         // pend0_q_dot.y = (pend0_q.y - pend0_q_last.y) / diff_time;
//         // pend0_q_dot.z = (pend0_q.z - pend0_q_last.z) / diff_time;
//         // pend1_q_dot.x = (pend1_q.x - pend1_q_last.x) / diff_time;
//         // pend1_q_dot.y = (pend1_q.y - pend1_q_last.y) / diff_time;
//         // pend1_q_dot.z = (pend1_q.z - pend1_q_last.z) / diff_time;
//         pend0_q_dot = applyFiniteDiffVector3(pend0_q, pend0_q_last, diff_time);
//         pend1_q_dot = applyFiniteDiffVector3(pend1_q, pend1_q_last, diff_time);
//         pend0_q_dot_lpf1 = applyLPFVector3(pend0_q, pend0_q_last, pend0_q_last_2, pend0_q_dot_lpf1_last, pend0_q_dot_lpf1_last_2, diff_time, lpf_tau, lpf_xi, lpf_omega, 1);
//         pend1_q_dot_lpf1 = applyLPFVector3(pend1_q, pend1_q_last, pend1_q_last_2, pend1_q_dot_lpf1_last, pend1_q_dot_lpf1_last_2, diff_time, lpf_tau, lpf_xi, lpf_omega, 1);
//         pend0_q_dot_lpf2 = applyLPFVector3(pend0_q, pend0_q_last, pend0_q_last_2, pend0_q_dot_lpf2_last, pend0_q_dot_lpf2_last_2, diff_time, lpf_tau, lpf_xi, lpf_omega, 2);
//         pend1_q_dot_lpf2 = applyLPFVector3(pend1_q, pend1_q_last, pend1_q_last_2, pend1_q_dot_lpf2_last, pend1_q_dot_lpf2_last_2, diff_time, lpf_tau, lpf_xi, lpf_omega, 2);
        
//     }    

//     pend0_omega = crossProduct(pend0_q, pend0_q_dot);
//     pend1_omega = crossProduct(pend1_q, pend1_q_dot);

//     // LPF debug msg
//     lpf_data.header.stamp = ros::Time::now();
//     lpf_data.pend0_omega_lpf1 = crossProduct(pend0_q, pend0_q_dot_lpf1);
//     lpf_data.pend1_omega_lpf1 = crossProduct(pend1_q, pend1_q_dot_lpf1);
//     lpf_data.pend0_omega_lpf2 = crossProduct(pend0_q, pend0_q_dot_lpf2);
//     lpf_data.pend1_omega_lpf2 = crossProduct(pend1_q, pend1_q_dot_lpf2);

//     // next step
//     pend0_q_last_2 = pend0_q_last;
//     pend1_q_last_2 = pend1_q_last;

//     pend0_q_last = pend0_q;
//     pend1_q_last = pend1_q;

//     pend0_q_dot_lpf1_last_2 = pend0_q_dot_lpf1_last;
//     pend1_q_dot_lpf1_last_2 = pend1_q_dot_lpf1_last;
//     pend0_q_dot_lpf2_last_2 = pend0_q_dot_lpf2_last;
//     pend1_q_dot_lpf2_last_2 = pend1_q_dot_lpf2_last; 

//     pend0_q_dot_lpf1_last = pend0_q_dot_lpf1;
//     pend1_q_dot_lpf1_last = pend1_q_dot_lpf1;
//     pend0_q_dot_lpf2_last = pend0_q_dot_lpf2;   
//     pend1_q_dot_lpf2_last = pend1_q_dot_lpf2;       

//     // Directly Use Gazebo omegas:
//     if(gazebo_omega_enabled){
//         pend0_omega.x = (msg -> twist[pend0_link_index]).angular.x;
//         pend0_omega.y = -(msg -> twist[pend0_link_index]).angular.y;
//         pend0_omega.z = -(msg -> twist[pend0_link_index]).angular.z;
//         pend1_omega.x = (msg -> twist[pend1_link_index]).angular.x;
//         pend1_omega.y = -(msg -> twist[pend1_link_index]).angular.y;
//         pend1_omega.z = -(msg -> twist[pend1_link_index]).angular.z;
//     }


//     // system state msg
//     state18.header.stamp = ros::Time::now();
//     state18.state18[0] = load_pose.pose.position.x;
//     state18.state18[1] = load_pose.pose.position.y;
//     state18.state18[2] = load_pose.pose.position.z;
//     state18.state18[3] = pend0_q.x;
//     state18.state18[4] = pend0_q.y;
//     state18.state18[5] = pend0_q.z;
//     state18.state18[6] = pend1_q.x;
//     state18.state18[7] = pend1_q.y;
//     state18.state18[8] = pend1_q.z;
//     state18.state18[9] = load_twist.twist.linear.x;
//     state18.state18[10] = load_twist.twist.linear.y;
//     state18.state18[11] = load_twist.twist.linear.z; 
//     state18.state18[12] = pend0_omega.x;
//     state18.state18[13] = pend0_omega.y;
//     state18.state18[14] = pend0_omega.z;
//     state18.state18[15] = pend1_omega.x;
//     state18.state18[16] = pend1_omega.y;
//     state18.state18[17] = pend1_omega.z;   
    

//     //publisher
//     if(dea_enabled){
//         // ROS_INFO_STREAM("Running DEA...");
//         if(open_loop_ctrl_enabled) applyOpenLoopController();
//         else applyDSLSDEAController(state18, dea_xi4, dea_k, dea_param, dsls_dea_ref, ros::Time::now().toSec() - dea_start_time);
//         attitude_dea_0.header.stamp = ros::Time::now();
//         attitude_dea_1.header.stamp = ros::Time::now();
//         attitude_setpoint_pub_0->publish(attitude_dea_0); 
//         attitude_setpoint_pub_1->publish(attitude_dea_1); 
//     }
//     else{
//         // ROS_INFO_STREAM("Running Quad...");
//         applyQuad0Controller(Kv6, setpoint_0); 
//         applyQuad1Controller(Kv6, setpoint_1); 
//         attitude_0.header.stamp = ros::Time::now();
//         attitude_1.header.stamp = ros::Time::now();
//         attitude_setpoint_pub_0->publish(attitude_0);
//         attitude_setpoint_pub_1->publish(attitude_1);
//         applyDSLSDEAController(state18, dea_xi4, dea_k, dea_param, dsls_dea_ref, 0);
//     }
    
// }

// void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav){
//     double attctrl_tau = 0.1;
//     double thrust_norm_hover = 0.538;
//     double thrust_coeff = 100;  
//     double thrust_0 = 1.55*9.81;
//     double max_force = (uav_mass + 0.5 * load_mass) * max_fb_acc * 2; //
//     double force_norm = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);

//     /* Attitude */
    
//     // Getting Current Attitude
//     geometry_msgs::Quaternion quat_msg;
//     double curr_roll, curr_pitch, curr_yaw;    
//     if(uav == 0) quat_msg = uav0_pose.pose.orientation;
//     else if(uav == 1) quat_msg = uav1_pose.pose.orientation;
//     tf2::Quaternion quat_tf;
//     tf2::fromMsg(quat_msg, quat_tf);
//     tf2::Matrix3x3(quat_tf).getRPY(curr_roll, curr_pitch, curr_yaw);
//     bool curr_rpy_debug_enabled = false;
//     if(curr_rpy_debug_enabled) ROS_INFO_STREAM("Current RPY:" << curr_roll << " " << curr_pitch << " " << curr_yaw);

//     // Clip reference force
//     bool force_clip_enabled = true;
//     if(force_norm > max_force && force_clip_enabled){
//         // ROS_INFO_STREAM("[f2r] Force Trimming Detected, Norm: " << force_norm);
//         for(int i=0; i<3; i++) controller_output[i] *= max_force / force_norm;
//     }


//     // Getting ENU Attitude Target (mavros will do the conversion to NED)
//     attitude.header.stamp = ros::Time::now();
//     double ref_roll, ref_pitch, ref_yaw, ref_thrust;
//     ref_thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
//     ref_roll = std::asin((std::cos(curr_yaw) * controller_output[1] - std::sin(curr_yaw) * controller_output[0])/ref_thrust);
//     ref_pitch = std::atan2(std::cos(curr_yaw) * controller_output[0] + std::sin(curr_yaw) * controller_output[1], -controller_output[2]);
//     ref_yaw = 0;

//     tf2::Quaternion attitude_target_q;
//     bool ref_rpy_debug_enabled = false;
//     if(ref_rpy_debug_enabled) ROS_INFO_STREAM("Target RPY:" << ref_roll << " " << ref_pitch << " " << ref_yaw);
//     attitude_target_q.setRPY(ref_roll, ref_pitch, ref_yaw);
//     attitude.orientation.x = attitude_target_q.getX();
//     attitude.orientation.y = attitude_target_q.getY();
//     attitude.orientation.z = attitude_target_q.getZ();
//     attitude.orientation.w = attitude_target_q.getW();


//     /* Rate */

//     Eigen::Vector4d curr_att;
//     Eigen::Vector4d ref_att;

//     if (uav == 0){
//         curr_att(0) = uav0_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
//         curr_att(1) = uav0_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
//         curr_att(2) = uav0_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
//         curr_att(3) = uav0_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
//     }
//     else if (uav == 1){
//         curr_att(0) = uav1_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
//         curr_att(1) = uav1_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
//         curr_att(2) = uav1_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
//         curr_att(3) = uav1_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
//     }
//     ref_att(0) = attitude_target_q.getW();
//     ref_att(1) = attitude_target_q.getX();
//     ref_att(2) = attitude_target_q.getY();
//     ref_att(3) = attitude_target_q.getZ();

//     const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
//     const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
//     const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); // ROS_INFO_STREAM("qe:" << qe);
//     attitude.body_rate.x = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(1);
//     attitude.body_rate.y = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(2);
//     attitude.body_rate.z = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(3);   

//     // ROS_INFO_STREAM("body_rate_x:" << attitude.body_rate.x << " desired_rate_(0):" << desired_rate_(0));
//     // ROS_INFO_STREAM("body_rate_y:" << attitude.body_rate.y << " desired_rate_(1):" << desired_rate_(1));
//     // ROS_INFO_STREAM("body_rate_z:" << attitude.body_rate.z << " desired_rate_(2):" << desired_rate_(2));

//     /* Thrust */

//     attitude.thrust = std::max(0.0, std::min(1.0, (ref_thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
//     attitude.type_mask = 128;
//     // attitude.type_mask = 1|2|4;
// }

// geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, const geometry_msgs::Vector3 v2) {
//     geometry_msgs::Vector3 result;
//     result.x = v1.y * v2.z - v1.z * v2.y; 
//     result.y = v1.z * v2.x - v1.x * v2.z; 
//     result.z = v1.x * v2.y - v1.y * v2.x; 
//     return result; 
// }

// void applyQuad0Controller(double Kv6[6], double setpoint[6]){
//     double M_QUAD = 1.55;
//     double controller_output[3];
//     controller_output[0] = (-Kv6[0] * (uav0_pose.pose.position.x - setpoint[0]) - Kv6[3] * (uav0_twist.twist.linear.x - setpoint[3]))*M_QUAD;
//     controller_output[1] = (-Kv6[1] * (uav0_pose.pose.position.y - setpoint[1]) - Kv6[4] * (uav0_twist.twist.linear.y - setpoint[4]))*M_QUAD;
//     controller_output[2] = (-Kv6[2] * (uav0_pose.pose.position.z - setpoint[2]) - Kv6[5] * (uav0_twist.twist.linear.z - setpoint[5]) - 9.81)*M_QUAD;
//     force_rate_convert(controller_output, attitude_0, 0);
// }

// void applyQuad1Controller(double Kv6[6], double setpoint[6]){
//     double M_QUAD = 1.55;
//     double controller_output[3];
//     controller_output[0] = (-Kv6[0] * (uav1_pose.pose.position.x - setpoint[0]) - Kv6[3] * (uav1_twist.twist.linear.x - setpoint[3]))*M_QUAD;
//     controller_output[1] = (-Kv6[1] * (uav1_pose.pose.position.y - setpoint[1]) - Kv6[4] * (uav1_twist.twist.linear.y - setpoint[4]))*M_QUAD;
//     controller_output[2] = (-Kv6[2] * (uav1_pose.pose.position.z - setpoint[2]) - Kv6[5] * (uav1_twist.twist.linear.z - setpoint[5]) - 9.81)*M_QUAD;
//     force_rate_convert(controller_output, attitude_1, 1);
// }

// void applyDEAController(
//     double_sls_controller::DSlsState state18, 
//     double_sls_controller::DEAState &dea_xi4, 
//     const double dea_k[24], 
//     const double dea_param[4], 
//     const double dea_ref[13],
//     double t
//     ){

//     /* Getting Full State */
//     double state22[22] = {};
//     for(int i = 0; i < 22; i++){
//         if(i < 18) state22[i] = state18.state18[i];
//         else if(i < 22) state22[i] = dea_xi4.dea_xi4[i-18];
//     }

//     /* Apply Controller */
//     double F1[3];
//     double F2[3];
//     double xi_dot[4];
//     double sys_output[24];
//     DEAController(state22, dea_k, dea_param, dea_ref, t, F1, F2, xi_dot, sys_output);
//     dea_xi4.header.stamp = ros::Time::now();
//     for(int i = 0; i < 4; i ++){
//         dea_xi4.dea_xi_dot4[i] = xi_dot[i];
//     }

//     /* Infill DEA Force Msg */
//     dea_force_0.header.stamp = ros::Time::now();
//     dea_force_0.vector.x = F1[0];
//     dea_force_0.vector.y = F1[1];
//     dea_force_0.vector.z = F1[2];

//     dea_force_1.header.stamp = ros::Time::now();
//     dea_force_1.vector.x = F2[0];
//     dea_force_1.vector.y = F2[1];
//     dea_force_1.vector.z = F2[2];

//     force_rate_convert(F1, attitude_dea_0, 0);
//     force_rate_convert(F2, attitude_dea_1, 1);

//     /* Controller State Integration */
//     double diff_time;
//     bool reset_controller_state = false;
//     diff_time = (ros::Time::now().toSec() - controller_last_called);
//     controller_last_called = ros::Time::now().toSec(); 
//     if(dea_enabled){
//         for(int j = 0; j < 4; j ++){
//             dea_xi4.dea_xi4[j] += xi_dot[j]*diff_time; // Euler
//             if(std::isnan(dea_xi4.dea_xi4[j])){
//                 reset_controller_state = true;
//                 break;
//             } 
//         }         
//     }
 
//     if(reset_controller_state){
//         for(int i = 0; i<4; i++) dea_xi4.dea_xi4[i] = dea_xi4_ic[i];
//         ROS_INFO_STREAM("[applyDEA] xi reset detected");
//     }      


// }

// int applyDSLSDEAController(
//     double_sls_controller::DSlsState state18, 
//     double_sls_controller::DEAState &dea_xi4, 
//     const double dea_k[24], 
//     const double dea_param[4], 
//     const double dea_ref[15],
//     double t
//     ){

//     /* Getting Full State */
//     double state22[22] = {};
//     for(int i = 0; i < 22; i++){
//         if(i < 18) state22[i] = state18.state18[i];
//         else if(i < 22) state22[i] = dea_xi4.dea_xi4[i-18];
//     }

//     /* Apply Controller */
//     double F1[3];
//     double F2[3];
//     double xi_dot[4];
//     DSLSDEAController(state22, dea_k, dea_param, dea_ref, t, F1, F2, xi_dot);
//     dea_xi4.header.stamp = ros::Time::now();
//     for(int i = 0; i < 4; i ++){
//         dea_xi4.dea_xi_dot4[i] = xi_dot[i];
//     }

//     /* Infill DEA Force Msg */
//     dea_force_0.header.stamp = ros::Time::now();
//     dea_force_0.vector.x = F1[0];
//     dea_force_0.vector.y = F1[1];
//     dea_force_0.vector.z = F1[2];

//     dea_force_1.header.stamp = ros::Time::now();
//     dea_force_1.vector.x = F2[0];
//     dea_force_1.vector.y = F2[1];
//     dea_force_1.vector.z = F2[2];

//     force_rate_convert(F1, attitude_dea_0, 0);
//     force_rate_convert(F2, attitude_dea_1, 1);

//     /* Controller State Integration */
//     double diff_time;
//     bool reset_controller_state = false;
//     diff_time = (ros::Time::now().toSec() - controller_last_called);
//     controller_last_called = ros::Time::now().toSec(); 
//     if(dea_enabled){
//         for(int j = 0; j < 4; j ++){
//             dea_xi4.dea_xi4[j] += xi_dot[j]*diff_time; // Euler
//             if(std::isnan(dea_xi4.dea_xi4[j])){
//                 reset_controller_state = true;
//                 break;
//             } 
//         }         
//     }
 
//     if(reset_controller_state){
//         for(int i = 0; i<4; i++) dea_xi4.dea_xi4[i] = dea_xi4_ic[i];
//         ROS_INFO_STREAM("[applyDSLSDEA] xi reset detected");
//     }      

//     return 0;
// }


// // void getTargetAttitude(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav){
// //     //temp
// //     double attctrl_tau = 0.1;
// //     double thrust_norm_hover = 0.538;
// //     double thrust_coeff = 100;  
// //     double thrust_0 = uav_mass*9.81;
// //     double max_force = uav_mass * max_fb_acc;
// //     double force_norm = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
// //     if(force_norm > max_force){
// //         for(int i=0; i<3; i++) controller_output[0] *= max_force / force_norm; // Clip reference force
// //     }
       
// //     attitude.header.stamp = ros::Time::now();
// //     double ref_roll, ref_pitch, ref_yaw, ref_thrust;
// //     ref_thrust = sqrt(controller_output[0]*controller_output[0] + controller_output[1]*controller_output[1] + controller_output[2]*controller_output[2]);
// //     ref_yaw = 0;
// //     ref_roll = std::asin(controller_output[1]/ref_thrust);
// //     ref_pitch = std::atan2(controller_output[0], -controller_output[2]);
// //     tf2::Quaternion attitude_target_q;
// //     attitude_target_q.setRPY(ref_roll, ref_pitch, ref_yaw);
// //     // attitude.orientation.x = attitude_target_q.getX();
// //     // attitude.orientation.y = attitude_target_q.getY();
// //     // attitude.orientation.z = attitude_target_q.getZ();
// //     // attitude.orientation.w = attitude_target_q.getW();

// //     Eigen::Vector4d curr_att;
// //     Eigen::Vector4d ref_att;

// //     if (uav == 0){
// //         curr_att(0) = uav0_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
// //         curr_att(1) = uav0_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
// //         curr_att(2) = uav0_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
// //         curr_att(3) = uav0_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
// //     }
// //     else if (uav == 1){
// //         curr_att(0) = uav1_pose.pose.orientation.w; //ROS_INFO_STREAM("uav0_pose.pose.ori.w:" << uav0_pose.pose.orientation.w);
// //         curr_att(1) = uav1_pose.pose.orientation.x; //ROS_INFO_STREAM("uav0_pose.pose.ori.x:" << uav0_pose.pose.orientation.x);
// //         curr_att(2) = uav1_pose.pose.orientation.y; //ROS_INFO_STREAM("uav0_pose.pose.ori.y:" << uav0_pose.pose.orientation.y);
// //         curr_att(3) = uav1_pose.pose.orientation.z; //ROS_INFO_STREAM("uav0_pose.pose.ori.z:" << uav0_pose.pose.orientation.z);
// //     }
// //     ref_att(0) = attitude_target_q.getW();
// //     ref_att(1) = attitude_target_q.getX();
// //     ref_att(2) = attitude_target_q.getY();
// //     ref_att(3) = attitude_target_q.getZ();

// //     const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
// //     const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
// //     const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att); // ROS_INFO_STREAM("qe:" << qe);
// //     attitude.body_rate.x = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(1);
// //     attitude.body_rate.y = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(2);
// //     attitude.body_rate.z = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(3);   

// //     // ROS_INFO_STREAM("body_rate_x:" << attitude.body_rate.x << " desired_rate_(0):" << desired_rate_(0));
// //     // ROS_INFO_STREAM("body_rate_y:" << attitude.body_rate.y << " desired_rate_(1):" << desired_rate_(1));
// //     // ROS_INFO_STREAM("body_rate_z:" << attitude.body_rate.z << " desired_rate_(2):" << desired_rate_(2));


// //     attitude.ref_thrust = std::max(0.0, std::min(1.0, (ref_thrust - thrust_0) / thrust_coeff + thrust_norm_hover));
// //     attitude.type_mask = 128;
// // }

// int enu2ned(void){
//     uav0_pose.pose.position.x = uav0_pose.pose.position.y;
//     uav0_pose.pose.position.y = uav0_pose.pose.position.x; 
//     uav0_pose.pose.position.z = -uav0_pose.pose.position.z; 

//     uav1_pose.pose.position.x = uav1_pose.pose.position.y; 
//     uav1_pose.pose.position.y = uav1_pose.pose.position.x; 
//     uav1_pose.pose.position.z = -uav1_pose.pose.position.z; 

//     uav0_twist.twist.linear.x = uav0_twist.twist.linear.y;
//     uav0_twist.twist.linear.y = uav0_twist.twist.linear.x;
//     uav0_twist.twist.linear.z = -uav0_twist.twist.linear.z;

//     uav1_twist.twist.linear.x = uav1_twist.twist.linear.y;
//     uav1_twist.twist.linear.y = uav1_twist.twist.linear.x;
//     uav1_twist.twist.linear.z = -uav1_twist.twist.linear.z;

//     load_pose.pose.position.x = load_pose.pose.position.y; 
//     load_pose.pose.position.y = load_pose.pose.position.x; 
//     load_pose.pose.position.z = -load_pose.pose.position.z;    

//     load_twist.twist.linear.x = load_twist.twist.linear.y;
//     load_twist.twist.linear.y = load_twist.twist.linear.x;
//     load_twist.twist.linear.z = -load_twist.twist.linear.z;

//     return 0;
// }

// int enu2esd(void){
//     uav0_pose.pose.position.y = -uav0_pose.pose.position.y; 
//     uav0_pose.pose.position.z = -uav0_pose.pose.position.z; 

//     uav1_pose.pose.position.y = -uav1_pose.pose.position.y; 
//     uav1_pose.pose.position.z = -uav1_pose.pose.position.z; 

//     uav0_twist.twist.linear.y = -uav0_twist.twist.linear.y;
//     uav0_twist.twist.linear.z = -uav0_twist.twist.linear.z;

//     uav1_twist.twist.linear.y = -uav1_twist.twist.linear.y;
//     uav1_twist.twist.linear.z = -uav1_twist.twist.linear.z;

//     load_pose.pose.position.y = -load_pose.pose.position.y; 
//     load_pose.pose.position.z = -load_pose.pose.position.z;    

//     load_twist.twist.linear.y = -load_twist.twist.linear.y;
//     load_twist.twist.linear.z = -load_twist.twist.linear.z;

//     return 0;
// }

// int applyOpenLoopController(void){
    
//     return 0;
// }

// double applyFiniteDiff(double x, double x_last, double diff_time){
//     bool fd_debug_enabled = false;
//     double x_dot;
//     if(diff_time < DBL_MIN) {
//         printf("[FiniteDiff][Fatal]: Time Step Too Small\n");
//         return 0;
//     }
//     else{
//         x_dot = (x - x_last) / diff_time;
//         if(fd_debug_enabled) printf("[FiniteDiff] Finite Difference Complete\n");
//         return x_dot;
//     }
    
// }

// geometry_msgs::Vector3 applyFiniteDiffVector3(geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, double diff_time){
//     geometry_msgs::Vector3 v_dot;
//     v_dot.x = applyFiniteDiff(v.x, v_last.x, diff_time);
//     v_dot.y = applyFiniteDiff(v.y, v_last.y, diff_time);
//     v_dot.z = applyFiniteDiff(v.z, v_last.z, diff_time);
//     return v_dot;
// }

// double applyLPF(double x, double x_last, double x_last_2, double x_dot_last, double x_dot_last_2, double diff_time, double tau, double xi, double omega, int order){
//     bool lpf_debug_enabled = false;
//     double x_dot;
//     if(order == 1){ // 1st Order LPF
//         double T = diff_time; // / 1000.0f / 1000.0f;
//         if(T < DBL_MIN) return x_dot_last;
//         double a = 2 / (T + 2 * tau);
//         double b = (2 * tau - T) / (T + 2 * tau);
//         x_dot = a * (x - x_last) + b * x_dot_last;
//         if(lpf_debug_enabled) printf("[LPF] 1st Order LPF Complete\n");
//     }
//     else if(order == 2){ // 2nd Order LPF
//         double T = diff_time; // / 1000.0f / 1000.0f;
//         if(T < DBL_MIN) return x_dot_last;
//         double k1 = 4 + 4*xi*omega*T + omega*omega*T*T;
//         double k2 = -8 + 2 * omega*omega*T*T;
//         double k3 = 4 - 4 * xi*omega*T + omega*omega*T*T;
//         double k4 = 2*omega*omega*T;
//         x_dot = k4/k1*(x - x_last_2) - k2/k1*x_dot_last - k3/k1*x_dot_last_2;
//         if(lpf_debug_enabled) printf("[LPF] 2nd Order LPF Complete\n");
//     }
//     else {
//         printf("[LPF][Error] Order Error Detected\n");
//         return x_dot_last;
//     }

//     return x_dot;
// }

// geometry_msgs::Vector3 applyLPFVector3(
//     geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, geometry_msgs::Vector3 v_last_2, geometry_msgs::Vector3 v_dot_last, geometry_msgs::Vector3 v_dot_last_2, 
//     double diff_time, double tau, double xi, double omega, int order
//     ){
//     geometry_msgs::Vector3 v_dot;
//     v_dot.x = applyLPF(v.x, v_last.x, v_last_2.x, v_dot_last.x, v_dot_last_2.x, diff_time, tau, xi, omega, order);
//     v_dot.y = applyLPF(v.x, v_last.y, v_last_2.y, v_dot_last.y, v_dot_last_2.y, diff_time, tau, xi, omega, order);
//     v_dot.z = applyLPF(v.x, v_last.z, v_last_2.z, v_dot_last.z, v_dot_last_2.z, diff_time, tau, xi, omega, order);
//     return v_dot;
// }





// // void geometricCtrl::pubSystemStatus() {
// //   mavros_msgs::CompanionProcessStatus msg;

// //   msg.header.stamp = ros::Time::now();
// //   msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
// //   msg.state = (int)companion_state_;

// //   systemstatusPub_.publish(msg);
// // }


# include "double_sls_controller/double_sls_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "double_sls_controller");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");    
    dslsCtrl* dslsController = new dslsCtrl(nh, nh_private);

    dynamic_reconfigure::Server<double_sls_controller::DoubleSLSControllerConfig> srv;
    dynamic_reconfigure::Server<double_sls_controller::DoubleSLSControllerConfig>::CallbackType f;
    f = boost::bind(&dslsCtrl::dynamicReconfigureCallback, dslsController, _1, _2);
    srv.setCallback(f);
    
    ros::spin();
    return 0;
}

