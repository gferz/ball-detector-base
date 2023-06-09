/**
 * @file ball_detector.hpp
 * @author ALFAROBI V13
 * @brief Base class for ball detector
 * @version 0.1
 * @date 2023-06-07
 * 
 * 
*/

#pragma once

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

namespace alfarobi::vision
{
    /**
     * @brief Base class for ball detector
     * 
     */
    class BallDetector
    {
        protected:
            /**
             * @brief Ros node handle.
             * 
             */
            ros::NodeHandle nh_;

            /**
             * @brief Main frame from the image input.
             * 
             */
            cv::Mat main_frame;

            /**
             * @brief Publish ball coordinate.
             * 
             * @param x x coordinate of ball
             * @param y y coordinate of ball
             */
            virtual void publishCoordinate(int x, int y) const final;

            /**
             * @brief Ball not found.
             * 
             */
            virtual void publishNotFound() const final;

            /**
             * @brief This method will only be called once.
             * 
             */
            virtual void setup();

            /**
             * @brief This method will be called in the loop.
             * 
             */
            virtual void update();

        private:
            image_transport::ImageTransport it_;
            image_transport::Subscriber sub_;

            ros::Publisher pub_;

            void imageCallback(const sensor_msgs::ImageConstPtr& msg);
            
            int frame_height;
            int frame_width;

        public:
            BallDetector();
            ~BallDetector();

            /**
             * @brief Main loop method.
             * 
             */
            virtual void process() final;


            /**
             * @brief Set the frame_height value
             * 
             * @param height y size of frame
             */
            void setFrameHeight(int height);

            /**
             * @brief Set the frame_width value
             * 
             * @param width
             */
            void setFrameWidth(int width);
    };
}