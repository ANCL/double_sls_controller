#include <double_sls_controller/double_sls_controller.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>

// DoubleSLSController::DoubleSLSController(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) : nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE){
//     mav_state_sub_ = nh_.subscribe<mavros_msgs::State>
//             ("mavros/state", 10, &DoubleSLSController::mav_state_cb, this, ros::TransportHints().tcpNoDelay());
//     local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
//             ("mavros/setpoint_position/local", 10);
//     arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
//             ("mavros/cmd/arming");
//     set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
//             ("mavros/set_mode");

//     statusloop_timer_ = nh_.createTimer(ros::Duration(1), &DoubleSLSController::statusloopCallback,
//                                       this);  // Define timer for constant loop rate

    

//     //the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);
//     // wait for FCU connection
//     while(ros::ok() && !current_state_.connected){
//         ros::spinOnce();
//         rate.sleep();
//     }
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 2;


//     //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub_.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }



//     // local_pos_pub_.publish(pose);
//     // rate.sleep();

// }