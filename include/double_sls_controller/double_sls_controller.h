#ifndef DOUBLE_SLS_CONTROLLER_H
#define DOUBLE_SLS_CONTROLLER_H

#include <ros/ros.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>
#include <cmath>
#include <iostream>
#include <cstddef>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <double_sls_controller/configConfig.h>
#include <double_sls_controller/common.h>
#include <double_sls_controller/control.h>
#include <double_sls_controller/DSlsState.h>
#include <double_sls_controller/DEAState.h>
#include <double_sls_controller/LPFData.h>

#include <double_sls_controller/DoubleSLSControllerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define SITL_ENABLED true
#if SITL_ENABLED
    #include <gazebo_msgs/LinkStates.h>
    #include <gazebo_msgs/GetLinkState.h>
#endif

#define PI 3.1415926535
#define MIN_DELTA_TIME 1e-16

#if SITL_ENABLED
    void gazeboCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
#endif

bool gotime;

double dea_k[24];

void callback(double_sls_controller::configConfig &config, uint32_t level);

#endif

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class dslsCtrl {
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber state_sub_0_;
        ros::Subscriber state_sub_1_;
        ros::Subscriber gazebo_state_sub_;
        ros::Publisher local_pos_pub_0_;
        ros::Publisher local_pos_pub_1_;
        ros::Publisher attitude_setpoint_pub_0_;
        ros::Publisher attitude_setpoint_pub_1_;
        ros::Publisher test_setpoint_0_pub_;
        ros::Publisher test_setpoint_1_pub_;
        ros::Publisher dea_force_0_pub_;
        ros::Publisher dea_force_1_pub_; 
        ros::Publisher dsls_state_pub_;
        ros::Publisher lpf_data_pub_;
        ros::Publisher dea_state_pub_;
        ros::ServiceClient arming_client_0_;
        ros::ServiceClient arming_client_1_;
        ros::ServiceClient set_mode_client_0_;
        ros::ServiceClient set_mode_client_1_;
        ros::ServiceClient gz_link_state_client_;
        ros::ServiceClient set_ref_client_;
        ros::Timer cmdloop_timer_, statusloop_timer_;
        ros::Time last_request_, reference_request_now_, reference_request_last_;

        /* booleans */
        bool dea_enabled_ = false;
        bool dea_last_status_ = false;
        bool sitl_enabled_ = true;
        bool param_tuning_enabled_ = false;
        bool dynamic_reconfigure_enabled_ = false;
        bool use_ned_frame_ = false;
        bool open_loop_ctrl_enabled_ = false;
        bool gazebo_omega_enabled_ = false;
        bool att_saturation_enabled_ = false; // not used
        bool force_clip_enabled_ = false;
        bool lpf_debug_enabled_ = false;
        bool time_sync_debug_enabled_ = false;
        bool dea_controller_enabled_ = false;
        bool dea_preintegrate_enabled_ = false;

        /* physical parameters */
        double max_fb_acc_ = 10.0;
        double uav_mass_ = 1.56;
        double load_mass_ = 0.25;
        double cable_length_ = 0.85;
        double gravity_acc_ = 9.80665;
        double max_thrust_force_ = 31.894746920044025;
        double throttle_offset_ = 0.0;
        const double dea_param_[4] = {load_mass_, uav_mass_, cable_length_, gravity_acc_};
        double max_tilt_angle_ = 0.78598163; //45 deg
        double max_xi_value_ = 1e5;
        double att_ctrl_tau_ = 0.8;

        /* Reference Trajectory & Pend Angle */
        double radium_scalar_ = 0;
        double freq_scalar_ = 0;
        double r_1_ = 5.0 ;
        double fr_1_ = 1.0;
        double c_1_ = 0;
        double ph_1_ = 0;
        double r_2_ = 5.0 ;
        double fr_2_ = 1.0;
        double c_2_ = 0.0;
        double ph_2_ = PI/2;
        double r_3_ = 0;
        double fr_3_ = 0.0;
        double c_3_ = -2.0;
        double ph_3_ = 0;
        double pend_angle_deg_ = 90; // deprecated
        double q_1_3_r_ = 0.7406322196911044; //sqrt(2)/2;
        double q_2_1_r_ = 0;
        double q_2_2_r_ = -0.6717825272800765; //-sqrt(2)/2;
        double dsls_dea_ref_[15] = {r_1_ * radium_scalar_, fr_1_ * freq_scalar_, c_1_, ph_1_, 
                                    r_2_ * radium_scalar_, fr_2_ * freq_scalar_, c_2_, ph_2_, 
                                    r_3_ * radium_scalar_, fr_3_ * freq_scalar_, c_3_, ph_3_, 
                                    q_1_3_r_, q_2_1_r_, q_2_2_r_};
        double dsls_dea_ref_temp_[15];

        /* Gains */
        double dea_k1_[4] = {31.6228,   37.4556,   20.6010,    6.4963};
        double dea_k2_[4] = {31.6228,   37.4556,   20.6010,    6.4963};
        double dea_k3_[4] = {24.0000,   50.0000,   35.0000,   10.0000};
        double dea_k4_[4] = {2.0000,    3.0000,         0,         0};
        double dea_k5_[4] = {2.0000,    3.0000,         0,         0};
        double dea_k6_[4] = {2.0000,    3.0000,         0,         0};
        double dea_k_[24];

        /* lpf parameters*/
        double lpf_tau_;
        double lpf_xi_;
        double lpf_omega_;

        /* (Primed) Initial Conditions */
        double load_pose_ic_[3] = {0, 0, c_3_};
        double setpoint_0_[6] = {load_pose_ic_[0], load_pose_ic_[1] - cable_length_*sin(pend_angle_deg_ * 0.5),  load_pose_ic_[2] - cable_length_*cos(pend_angle_deg_ * 0.5), 0, 0, 0};
        double setpoint_1_[6] = {load_pose_ic_[0], load_pose_ic_[1] + cable_length_*sin(pend_angle_deg_ * 0.5),  load_pose_ic_[2] - cable_length_*cos(pend_angle_deg_ * 0.5), 0, 0, 0};
        double dea_xi4_ic_[4] = {-gravity_acc_, -5.10, 0, 0}; 

        /* Gains */
        double Kv6_[6] = {4.3166, 4.3166, 4.316, 3.1037, 3.1037, 3.1037};

        /* Time */
        double gazebo_last_called_;
        double controller_last_called_;
        double dea_start_time_; 
        double dea_end_time_; 

        /* Gazebo Index Matching */
        bool gazebo_link_name_matched_ = false;
        int uav0_link_index_;
        int uav1_link_index_;
        int pend0_link_index_;
        int pend1_link_index_;
        int load_link_index_;
        const char* link_name_[5] = {
            "px4vision_0::base_link", 
            "px4vision_1::base_link",
            "slung_load::pendulum_0::base_link",
            "slung_load::pendulum_1::base_link",
            "slung_load::load::base_link"
            };


        /* Mission */
        bool mission_enabled_ = false;
        bool mission_start_ = false;
        bool mission_ref_updated_ = false;
        int mission_stage_ = 0;
        double mission_time_last_ = 0;
        double dea_mission_setpoint0_[6] = {0.0, 0.0, -2.0, q_1_3_r_, q_2_1_r_, q_2_2_r_};
        double dea_mission_setpoint1_[6] = {1.0, 0.0, -2.0, q_1_3_r_, q_2_1_r_, q_2_2_r_};
        double dea_mission_setpoint2_[6] = {0.0, 1.0, -2.0, q_1_3_r_, q_2_1_r_, q_2_2_r_};
        double dea_mission_setpoint3_[6] = {0.0, 0.0, -2.5, q_1_3_r_, q_2_1_r_, q_2_2_r_};
        double dea_mission_trajectory_[15] = {
            1.0, 0.5, 0.0, 0.0,
            1.0, 0.5, 0.0, PI/2,
            0.0, 0.0, -2.0, 0.0,
            q_1_3_r_, q_2_1_r_, q_2_2_r_
        };


        std::string uav0_link_name_ = "px4vision_0::base_link";
        std::string uav1_link_name_ = "px4vision_1::base_link";
        std::string pend0_link_name_ = "slung_load::pendulum_0::base_link";
        std::string pend1_link_name_ = "slung_load::pendulum_1::base_link";
        std::string load_link_name_ = "slung_load::load::base_link";
        gazebo_msgs::GetLinkState gz_link_state_request_;

        mavros_msgs::State current_state_0_;
        mavros_msgs::State current_state_1_;
        mavros_msgs::AttitudeTarget attitude_0_;
        mavros_msgs::AttitudeTarget attitude_1_;
        mavros_msgs::AttitudeTarget attitude_dea_0_;
        mavros_msgs::AttitudeTarget attitude_dea_1_;
        double_sls_controller::DSlsState state18_;
        double_sls_controller::DEAState dea_xi4_;
        double_sls_controller::LPFData lpf_data_;

        /* uav0 */
        geometry_msgs::PoseStamped uav0_pose_, uav0_pose_last_;
        geometry_msgs::TwistStamped uav0_twist_, uav0_twist_last_;
        /* uav1 */
        geometry_msgs::PoseStamped uav1_pose_, uav1_pose_last_;
        geometry_msgs::TwistStamped uav1_twist_, uav1_twist_last_;
        /* pend0 */
        geometry_msgs::Vector3 pend0_q_, pend0_q_last_, pend0_q_last_2_;
        geometry_msgs::Vector3 pend0_q_dot_, pend0_q_dot_last_, pend0_q_dot_last_2_;
        geometry_msgs::Vector3 pend0_q_dot_lpf1_, pend0_q_dot_lpf1_last_, pend0_q_dot_lpf1_last_2_;
        geometry_msgs::Vector3 pend0_q_dot_lpf2_, pend0_q_dot_lpf2_last_, pend0_q_dot_lpf2_last_2_;
        geometry_msgs::Vector3 pend0_omega_;
        /* pend1 */
        geometry_msgs::Vector3 pend1_q_, pend1_q_last_, pend1_q_last_2_;
        geometry_msgs::Vector3 pend1_q_dot_, pend1_q_dot_last_, pend1_q_dot_last_2_;
        geometry_msgs::Vector3 pend1_q_dot_lpf1_, pend1_q_dot_lpf1_last_, pend1_q_dot_lpf1_last_2_;
        geometry_msgs::Vector3 pend1_q_dot_lpf2_, pend1_q_dot_lpf2_last_, pend1_q_dot_lpf2_last_2_;
        geometry_msgs::Vector3 pend1_omega_;
        /* load */
        geometry_msgs::PoseStamped load_pose_, load_pose_last_;
        geometry_msgs::TwistStamped load_twist_;
        /* DEA Controller Output Force */
        geometry_msgs::Vector3Stamped dea_force_0_;
        geometry_msgs::Vector3Stamped dea_force_1_;

        void stateCb_0(const mavros_msgs::State::ConstPtr& msg);
        void stateCb_1(const mavros_msgs::State::ConstPtr& msg);
        void gazeboCb(const gazebo_msgs::LinkStates::ConstPtr& msg);
        void force_rate_convert(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav);
        void applyQuad0Controller(double Kv6[6], double setpoint[6]);
        void applyQuad1Controller(double Kv6[6], double setpoint[6]);
        int applyOpenLoopController(void);
        int applyDSLSDEAController(double_sls_controller::DSlsState state18, double_sls_controller::DEAState &dea_xi4, const double dea_k[24], const double dea_param[4], const double ref[15], double t);
        void getTargetAttitude(double controller_output[3], mavros_msgs::AttitudeTarget &attitude, int uav);
        geometry_msgs::Vector3 crossProduct(const geometry_msgs::Vector3 v1, const geometry_msgs::Vector3 v2);

        double applyLPF(double u, double u_last, double u_last_2, double y_last, double y_last_2, double Ts, double tau, double xi, double omega, int order);
        geometry_msgs::Vector3 applyLPFVector3(
            geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, geometry_msgs::Vector3 v_last_2, geometry_msgs::Vector3 v_dot_last, geometry_msgs::Vector3 v_dot_last_2, 
            double diff_time, double tau, double xi, double omega, int order
            );

        double applyFiniteDiff(double x, double x_last, double x_dot_last, double diff_time);
        geometry_msgs::Vector3 applyFiniteDiffVector3(geometry_msgs::Vector3 v, geometry_msgs::Vector3 v_last, geometry_msgs::Vector3 v_dot_last, double diff_time);
        int enu2ned(void);
        int enu2esd(void);
        void cmdloopCallback(const ros::TimerEvent &event);
        void statusloopCallback(const ros::TimerEvent &event);
        void pubDebugData(void);

        /* Service Functions */
        gazebo_msgs::LinkState getLinkState(std::string link_name);
        geometry_msgs::Pose getLinkPose(std::string link_name);
        geometry_msgs::Twist getLinkTwist(std::string link_name);

        enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;
        template <class T>
        void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
            ros::Rate pause(hz);
            ROS_INFO_STREAM(msg);
            while (ros::ok() && !(*pred)) {
                ros::spinOnce();
                pause.sleep();
            }
        };
        geometry_msgs::Pose home_pose_;
        bool received_home_pose;
        std::shared_ptr<Control> controller_;
        int executeMission(void);
        int checkMissionStage(double mission_time_span);
        int setDEASetpoint(double dea_setpoint[6]);
        int setDEATrajectory(double dea_trajectory[15]);
        int saveDSLSDEARef(void);
        int loadDSLSDEARef(void);


    public:
        void dynamicReconfigureCallback(double_sls_controller::DoubleSLSControllerConfig &config, uint32_t level);
        dslsCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~dslsCtrl();
};