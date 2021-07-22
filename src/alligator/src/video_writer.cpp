/* video_writer.cpp */

#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "ros_compat.h"

// #include "std_srvs/Empty.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  //  TODO  - Have to do some kind of persistant file index instead of relying on time which is only updated when connected to the internet!
  // return;
  // struct timespec initial_time, current_time;
  // clock_gettime(CLOCK_REALTIME, &initial_time);

  ros::init(argc, argv, "video_writer");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("jetcam/image", 1, imageCallback);

  cv::namedWindow("view");
  ros::spin();

  cv::destroyWindow("view");
  return 0;
}
