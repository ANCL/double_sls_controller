#include <rtwtypes.h>
#include <StabController.h>
#include <TracController.h>
#include <quasi_controller/quasi_controller.h>
#include <double_sls_controller/PTStates.h>
#include <double_sls_controller/AttOut.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,pose_get_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,vel_cb);
    #if QUASI_SITL_ENABLED
      ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 10, gazebo_state_cb);
    #else
      ros::Subscriber load_vicon_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/test_jul19/test_jul19",10, loadpose_cb);
    #endif
    ros::Publisher sls_state_pub = nh.advertise<double_sls_controller::PTStates>("/double_sls_controller/sls_state", 10);
    ros::Publisher target_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/double_sls_controller/target_attitude", 10);
    ros::Publisher att_con_pub = nh.advertise<double_sls_controller::AttOut>("/double_sls_controller/att_con", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Timer timer = nh.createTimer(ros::Duration(0.05), timerCallback);
    ros::Rate rate(50.0); //the setpoint publishing rate MUST be faster than 2Hz

    attitude.header.stamp = ros::Time::now();
    attitude.header.frame_id = "map"; 
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;
    attitude.orientation.w = 0;
    attitude.thrust = 0.2;
    ros::Time last_request = ros::Time::now();

    ROS_INFO("GET STATES NODE RUNNING");

    while(ros::ok()){   
        if(param_tuning_enabled){
          quasi_update_params(nh);
          quasi_print_params();
        }
        
        PT_state_pub(sls_state_pub);

        for (int i=0;i<10; i++){
          dv[i] = PTState.PT_states[i];
        }

        double temp = ros::Time::now().toSec();
        if(quad_only_enabled){
            apply_outerloop_control(Kv6, setpoint);    
        }
        else{
            StabController(dv, Kv12, Param, Setpoint, controller_output);
            if(rate_target_enabled){
                force_rate_convert(controller_output, attitude);
            }
            else{
                force_attitude_convert(controller_output, attitude);
            }            
        }
        attitude.header.stamp = ros::Time::now();
        target_attitude_pub.publish(attitude);
        printf("%f", ros::Time::now().toSec() -temp);
        
        if( current_state.mode != "OFFBOARD"){
          t0 = ros::Time::now().toSec();
        }
        
        if (track_debug_enabled){
          TracController(dv, Kv12, Param,  ros::Time::now().toSec()-t0, controller_output);
        }
      
        att_out_pub(att_con_pub, controller_output);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
