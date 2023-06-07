#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <ball_detector.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>

#define TOPIC "Topic Name Config"
#define FRAME "Frame Size Config"
#define CONFIG_PATH

namespace av = alfarobi::vision;

static YAML::Node *config = new YAML::Node(YAML::LoadFile(CONFIG_PATH));
static int frame_height = (*config)[FRAME]["frame height"].as<int>();
static int frame_width = (*config)[FRAME]["frame width"].as<int>();

av::BallDetector::BallDetector() :
it_(nh_),
sub_(it_.subscribe((*config)[TOPIC]["image input"].as<std::string>(), 1, &av::BallDetector::imageCallback,this)),
pub_(nh_.advertise<geometry_msgs::Point>((*config)[TOPIC]["ball coordinate"].as<std::string>(), 1000))
{
    delete config;
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

void av::BallDetector::publish_coordinate(int x, int y) const
{
    geometry_msgs::Point p;
    if(x < 0 || x > frame_height) throw("argument outside the range!");
    if(y < 0 || y > frame_height) throw("argument outside the range!");

    p.x = x;
    p.y = y;
    p.z = -1;

    pub_.publish(p);
}

void av::BallDetector::publish_not_found() const
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
    av::BallDetector::setup();

    while(ros::ok())
    { 
        if( !(main_frame.empty()) ) this->update();
        ros::spinOnce();
    }
}