#include <iostream>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// OpenCV
#include "OpenCV_Functions.h"
#include "OpenCV_Functions.cpp"
#include "OpenCV_Utils.h"
#include "OpenCV_Utils.cpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// etc
#include <pthread.h>
#include <queue>
//#include "ros_opencv_try/MsgACC.h"

cv::Mat image;

// Subscriber node
void imageCallback(const sensor_msgs::ImageConstPtr &msg) 
{
  try
  {
    image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::imshow("Cam1", image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_subscriber");
  ros::NodeHandle nh;

  cv::namedWindow("Cam1", WINDOW_NORMAL);
  cv::resizeWindow("Cam1", 640, 480);

  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/front_cam/image", 1, 
        imageCallback, image_transport::TransportHints("compressed"));

  ros::spin();
  cv::destroyWindow("Cam1");
}
