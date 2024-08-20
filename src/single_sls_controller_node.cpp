#include <rtwtypes.h>
#include <double_sls_controller/single_sls_controller.h>
#include <double_sls_controller/PTStates.h>
#include <double_sls_controller/AttOut.h>
#include <QuadSLS_PT_Controller_QSF.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "single_sls_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Subscriber mavros_state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, mavstateCallback);
    ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>
            ("/gazebo/link_states", 10, gazeboSingleSlsCallback);       
    ros::Publisher pose_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher attitude_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");  
    //ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,posegetCallback);
    //ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,velCallback);
    //ros::Publisher sls_state_pub = nh.advertise<double_sls_controller::PTStates>("/double_sls_controller/sls_state", 10);
    //ros::Publisher target_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/double_sls_controller/target_attitude", 10);
    //ros::Publisher att_con_pub = nh.advertise<double_sls_controller::AttOut>("/double_sls_controller/att_con", 10);
    //ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);
    ros::Rate rate(50.0); //the setpoint publishing rate MUST be faster than 2Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.25;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

       

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        pose_setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::SetMode land_mode;

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
	land_mode.request.custom_mode = "AUTO.LAND";

    ros::Time last_request = ros::Time::now();

    ROS_INFO("SINGLE SLS NODE RUNNNING"); 

    int stage = 0;

    while(ros::ok()){
        nh.getParam("custom_controller_enabled", custom_controller_enabled);
        // ROS_INFO_STREAM(current_state.mode);

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

        pose_setpoint_pub.publish(pose);





        switch (stage){  
            case 0: // takeoff
                if( current_state.mode != "OFFBOARD" &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) &&
                        offb_set_mode.response.mode_sent){
                        ROS_INFO("Offboard enabled");
                        ROS_INFO_STREAM(current_state.mode);
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

                pose.header.stamp = ros::Time::now();
                pose_setpoint_pub.publish(pose);

                err_pend_pose = std::pow((quad_1_pose.pose.position.x - pose.pose.position.x),2) 
                + std::pow((quad_1_pose.pose.position.y - pose.pose.position.y),2)
                + std::pow((quad_1_pose.pose.position.z - pose.pose.position.z),2);

                if(ros::Time::now() - last_request > ros::Duration(20.0) && err_pend_pose < 0.1){
                    ROS_INFO("Takeoff Complete, run \"rosparam set custom_controller_enabled true\" to enable custom controller");
                    if(custom_controller_enabled){
                        stage += 1;
                        last_request = ros::Time::now();
                        ROS_INFO("Single SLS Controller Enabled, Case Switched from 0 to 1");                
                    }
                }
                break;

            case 1: // setpoint position control
                single_sls_ref[0] = 0;
                single_sls_ref[1] = 0;
                single_sls_ref[2] = -2.0;
                QuadSLS_PT_Controller_QSF(single_sls_state, Kv12, single_sls_param, single_sls_ref, controller_output);
                // if(rate_target_enabled){
                //     force_rate_convert(controller_output, attitude);
                // }
                // else{
                //     force_attitude_convert(controller_output, attitude);
                // }
                // attitude.header.stamp = ros::Time::now();
                // attitude_target_pub.publish(attitude);
                updateSingleSlsTarget(controller_output, attitude);
                pubSingleSlsTarget(attitude_target_pub, attitude);

                err_pend_pose = std::pow((quad_1_pose.pose.position.x - single_sls_ref[0]),2) 
                    + std::pow((quad_1_pose.pose.position.y - (-single_sls_ref[1])),2)
                    + std::pow((quad_1_pose.pose.position.z - (-single_sls_ref[2]+L_CABLE)),2);
                if(ros::Time::now() - last_request > ros::Duration(15.0) && err_pend_pose < 0.2){
                    //stage += 1;
                    ROS_INFO("Target 1 Achieved, Case Switched from 1 to 2");
                    last_request = ros::Time::now();
                }
                break;

            }

            ros::spinOnce();
            rate.sleep();

        } 

    return 0;
}        

