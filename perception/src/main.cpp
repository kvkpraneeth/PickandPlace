#include "perception/vision.h"
#include "ros/node_handle.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv, "vision");

    //mnh_ -> Main Node Handle.
    ros::NodeHandle mnh_;

    ros::AsyncSpinner spinner(0);

    spinner.start();

    vision v(mnh_);

    ros::waitForShutdown();

    return 0;
}
