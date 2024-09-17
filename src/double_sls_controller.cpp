#include <double_sls_controller/double_sls_controller.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>

// void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){

//     uav0_pose.pose = msg->pose[7];
//     uav0_twist.twist = msg->twist[7];    
//     uav1_pose.pose = msg->pose[15];
//     uav1_twist.twist = msg->twist[15];   
//     // pendpose = msg->pose[9];    // 9: pendulum
//     // pendtwist = msg->twist[9];
//     // loadpose = msg->pose[10];   // 10: load; 
//     // loadtwist = msg->twist[10]; 

//     tf2::Quaternion quad_q(quadpose.orientation.x, quadpose.orientation.y, quadpose.orientation.z, quadpose.orientation.w);
//     tf2::Matrix3x3 quad_m(quad_q);
//     double quad_roll, quad_pitch, quad_yaw;
//     quad_m.getRPY(quad_roll, quad_pitch, quad_yaw);

//     get_quad_states();
//     get_pend_states();

//     updateLoadPose(msg);
//     updateQuadPose(msg);
//     updateLoadVel();
//     updateQuadVel();
//     updatePendAngle();
//     updateSingleSlsState();

//     // diff_time = (ros::Time::now().toSec() - gazebo_last_called);
//     // gazebo_last_called = ros::Time::now().toSec();

//     for(int i=0; i<10;i++){
//         ROS_INFO_STREAM(single_sls_state[i]);
//     }
// }

void callback(double_sls_controller::configConfig &config, uint32_t level) {
//    dea_k[0] = config.dea_k_0;
//    dea_k[1] = config.dea_k_1;
//    dea_k[2] = config.dea_k_2;
//    dea_k[3] = config.dea_k_3;
//    dea_k[4] = config.dea_k_4;
//    dea_k[5] = config.dea_k_5;
//    dea_k[6] = config.dea_k_6;
//    dea_k[7] = config.dea_k_7;
//    dea_k[8] = config.dea_k_8;
//    dea_k[9] = config.dea_k_9;
//    dea_k[10] = config.dea_k_10;
//    dea_k[11] = config.dea_k_11;
//    dea_k[12] = config.dea_k_12;
//    dea_k[13] = config.dea_k_13;
//    dea_k[14] = config.dea_k_14;
//    dea_k[15] = config.dea_k_15;
//    dea_k[16] = config.dea_k_16;
//    dea_k[17] = config.dea_k_17;
//    dea_k[18] = config.dea_k_18;
//    dea_k[19] = config.dea_k_19;
//    dea_k[20] = config.dea_k_20;
//    dea_k[21] = config.dea_k_21;
//    dea_k[22] = config.dea_k_22;
//    dea_k[23] = config.dea_k_23;

//    for(int j = 0; j < 24; j++){
//        ROS_INFO_STREAM(dea_k[j]);
//     }
}

