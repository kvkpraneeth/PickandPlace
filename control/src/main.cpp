#include "control/control.h"
#include "geometry_msgs/Pose.h"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv, "control");

    ros::AsyncSpinner spinner(0);

    spinner.start();

    control c;

    c.moveToGoal();

    ros::waitForShutdown();

    return 0;
}
