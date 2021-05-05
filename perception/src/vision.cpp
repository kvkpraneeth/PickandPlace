#include "perception/vision.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgproc.hpp"
#include "perception/getTransform.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#include <functional>
#include <memory>

vision::vision(ros::NodeHandle& nh_)
{
    this->nnh_ = nh_;
    
    this->cameraOutputSub = nnh_.subscribe(this->cameraOutputTopic, 1000, &vision::cameraOutputCallback, this);

    this->loopTimer = nnh_.createTimer(ros::Duration(0.01), &vision::timerCallback, this);

    // this->checkTransformPub.reset(new realtime_tools::RealtimePublisher<perception::checkTransform>(this->nnh_, this->checkTransformTopic, 1000));
}

void vision::cameraOutputCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    this->imagePtr = cv_bridge::toCvCopy(msg, msg->encoding);
    this->width = msg->width;
    this->height = msg->height;
}

void vision::calcTransform()
{
    cv::cvtColor(this->imagePtr->image, this->imagePtr->image, cv::COLOR_BGR2GRAY);
    
    cv::threshold(this->imagePtr->image, this->imagePtr->image, 127, 255, cv::THRESH_BINARY_INV);
    
    this->m = cv::moments(this->imagePtr->image, true);

    this->centroid = cv::Point(m.m10/m.m00, m.m01/m.m00);
}

void vision::timerCallback(const ros::TimerEvent &event)
{
    // this->checkTransformPub->unlockAndPublish();
}
