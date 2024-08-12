#include <quasi_controller/quasi_controller.h>
#include <double_sls_controller/PTStates.h>
#include <rtwtypes.h>
#include <StabController.h>
#include <TracController.h>


bool gotime;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,pose_get_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,vel_cb);
    ros::Subscriber attitude_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/double_sls_controller/target_attitude", 10, attitude_target_cb);
    ros::Subscriber sls_state_sub = nh.subscribe<double_sls_controller::PTStates>("/double_sls_controller/sls_state", 10, sls_state_cb);
    #if QUASI_SITL_ENABLED
      ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, gazebo_state_cb);
    #else
      ros::Subscriber load_vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/test_jul19/test_jul19",10, loadpose_cb);
    #endif
    ros::Publisher attitude_setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
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

    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map"; 
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 0;
    attitude.thrust = 0.2;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

	mavros_msgs::SetMode land_mode;
	land_mode.request.custom_mode = "AUTO.LAND";

    ros::Time last_request = ros::Time::now();

    int stage = 0;

    double temp;

    while(offb_ground_debug_enabled){
        quasi_update_params(nh);
        quasi_print_params();
    }

    while(ros::ok()){

        nh.getParam("bool", gotime);
        if(param_tuning_enabled){
            quasi_update_params(nh);    
        }

        for (int i=0; i<10; i++){
          dv[i] = PTState.PT_states[i];
        }
        
        switch (stage)
        {  
        case 0: // takeoff
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
            pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(pose);
            err_pend_pose = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
            + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
            + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);
            if(ros::Time::now() - last_request > ros::Duration(20.0) && err_pend_pose < 0.1){
                ROS_INFO("Quadrotor awaits the order to switch to quasi");
                if(gotime){
                    stage += 1;
                    last_request = ros::Time::now();
                    ROS_INFO("Takeoff finished and switch to position setpoint control mode");                
                }
            }
            break;

        case 1: // setpoint position control
            Setpoint[0] = 0.0;
            Setpoint[1] = 0.0;
            Setpoint[2] = -0.4;
            temp = ros::Time::now().toSec();
            ros::Duration(0.001).sleep();
            if(quad_only_enabled){
                apply_outerloop_control(Kv6, setpoint);    
                attitude.header.stamp = ros::Time::now();
                attitude_setpoint_pub.publish(attitude);    
                printf("%f", ros::Time::now().toSec() -temp);               
            }
            else{
                StabController(dv, Kv12, Param, Setpoint, controller_output);
                if(rate_target_enabled){
                    force_rate_convert(controller_output, attitude);
                }
                else{
                    force_attitude_convert(controller_output, attitude);
                }
                attitude.header.stamp = ros::Time::now();
                attitude_setpoint_pub.publish(attitude);
                printf("%f", ros::Time::now().toSec() -temp); 
            }



            err_pend_pose = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && err_pend_pose < 0.2){
                //stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 1");
                last_request = ros::Time::now();
            }
            break;

        case 2: // setpoint position control
            Setpoint[0] = 1.0;
            Setpoint[1] = 0.5;
            Setpoint[2] = -0.6;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);
            
            err_pend_pose = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            ROS_INFO_STREAM("err_pend_pose: " << err_pend_pose);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && err_pend_pose < 0.2){
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 2");
                last_request = ros::Time::now();
            }
            break;
        
        case 3: // setpoint position control
            Setpoint[0] = -1;
            Setpoint[1] = 0;
            Setpoint[2] = -0.3;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);
            
            err_pend_pose = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            ROS_INFO_STREAM("err_pend_pose: " << err_pend_pose);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && err_pend_pose < 0.2){
                stage += 1;
                ROS_INFO("Achieve position setpoint and switch to Setpoint 3");
                last_request = ros::Time::now();
            }
            break;

        case 4: // setpoint position control
            Setpoint[0] = 0;
            Setpoint[1] = 0;
            Setpoint[2] = -0.3;
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude.header.stamp = ros::Time::now();
            attitude_setpoint_pub.publish(attitude);
            
            err_pend_pose = std::pow((current_local_pos.pose.position.x - Setpoint[0]),2) 
            + std::pow((current_local_pos.pose.position.y - (-Setpoint[1])),2)
            + std::pow((current_local_pos.pose.position.z - (-Setpoint[2]+0.95)),2);
            ROS_INFO_STREAM("err_pend_pose: " << err_pend_pose);
            if(ros::Time::now() - last_request > ros::Duration(15.0) && err_pend_pose < 0.2){
                stage += 2;
                ROS_INFO("Achieve position setpoint and switch to Trajectory Tracking");
                last_request = ros::Time::now();
            }
            break;
        
        case 5: //Trajectory tracking
            attitude.header.stamp = ros::Time::now();
            TracController(dv, Kv12, Param, ros::Time::now().toSec() - last_request.toSec(), controller_output);
            force_attitude_convert(controller_output, attitude);
            attitude_setpoint_pub.publish(attitude);
            if(ros::Time::now() - last_request > ros::Duration(32.0)){
                stage += 1;
                ROS_INFO("Finish Trajactory Tracking and land");
                last_request = ros::Time::now();
            }
            break;


        case 6: // land
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 0.5;
            local_pos_pub.publish(pose);
            err_pend_pose = std::pow((current_local_pos.pose.position.x - pose.pose.position.x),2) 
            + std::pow((current_local_pos.pose.position.y - pose.pose.position.y),2)
            + std::pow((current_local_pos.pose.position.z - pose.pose.position.z),2);
            if(ros::Time::now() - last_request > ros::Duration(5.0)){
                if( set_mode_client.call(land_mode) && land_mode.response.mode_sent){
                    stage += 1;
                    ROS_INFO("Land finished");
                    // }
                }
            }
            break;
        
        default:
            break;
        }
		
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

