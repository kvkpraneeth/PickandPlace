#include "control/control.h"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "ros/node_handle.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv, "control");

    ros::NodeHandle mnh_;

    ros::AsyncSpinner spinner(0);

    spinner.start();

    control c(mnh_);

    ros::waitForShutdown();

    return 0;
}
