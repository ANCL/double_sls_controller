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