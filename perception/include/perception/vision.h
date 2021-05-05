#ifndef VISION_H
#define VISION_H

#include "ros/node_handle.h"
#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "cv_bridge/cv_bridge.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/subscriber.h"
#include <string>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
class vision
{
    //Constructor.
    public: vision(ros::NodeHandle &nh_);

    //Nodehandles.
    private: ros::NodeHandle nnh_;
    
    //Publications.
    // private: std::shared_ptr<realtime_tools::RealtimePublisher<perception::checkTransform>> checkTransformPub;

    //Crucial Topic Names.
    public: const std::string checkTransformTopic = "perceptionResult";
            const std::string cameraOutputTopic = "/camera/image_rect";

    //Subscriptions.    
    private: ros::Subscriber cameraOutputSub;

    //Callbacks.
    public: void cameraOutputCallback(const sensor_msgs::Image::ConstPtr& msg);
            void timerCallback(const ros::TimerEvent& event);

    //CameraInfo.
    public: const double fov = 1.3962634;
            int height=800;
            int width=800;

    //OutBringers.
    public: cv_bridge::CvImagePtr imagePtr;

    //Timers.
    public: ros::Timer loopTimer;

    //Worker Functions.
    public: void calcTransform();

    //OpenCv Variables.
    private: cv::Point centroid;
             cv::Moments m;

    //Messages.
    // private: perception::checkTransform::Ptr centroidTransform;

};

#endif
