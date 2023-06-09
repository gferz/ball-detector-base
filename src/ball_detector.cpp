#include <string>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ball_detector.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>

namespace av = alfarobi::vision;

av::BallDetector::BallDetector() :
it_(nh_)
{
    std::string img_in_topic = nh_.param<std::string>("image_input", "/usb_cam/image_raw");
    std::string ballpos_topic = nh_.param<std::string>("ball_pos", "/ball_pos");

    sub_ = it_.subscribe(img_in_topic, 1, &av::BallDetector::imageCallback,this);
    pub_ = nh_.advertise<geometry_msgs::Point>(ballpos_topic, 1000);

    frame_height = 640;
    frame_width = 640;
}

av::BallDetector::~BallDetector(){}

void av::BallDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        main_frame = (cv_bridge::toCvShare(msg, "bgr8")->image).clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void av::BallDetector::publishCoordinate(int x, int y) const
{
    geometry_msgs::Point p;
    if(x < 0 || x > frame_height) throw("argument outside the range!");
    if(y < 0 || y > frame_height) throw("argument outside the range!");

    p.x = x;
    p.y = y;
    p.z = -1;

    pub_.publish(p);
}

void av::BallDetector::publishNotFound() const
{
    geometry_msgs::Point p;

    p.x = -1;
    p.y = -1;
    p.z = -1;

    pub_.publish(p);
}

void av::BallDetector::setup(){}
void av::BallDetector::update(){}

void av::BallDetector::process()
{
    this->setup();

    while(ros::ok())
    { 
        if( !main_frame.empty() )
        {
            try
            {
                this->update();
            }
            catch(const std::exception& e)
            {
                ROS_ERROR("%s", e.what());
            }
        }

        ros::spinOnce();
    }
}

void av::BallDetector::setFrameHeight(int height)
{
    this->frame_height = height;
}

void av::BallDetector::setFrameWidth(int width)
{
    this->frame_width = width;
}