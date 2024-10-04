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
