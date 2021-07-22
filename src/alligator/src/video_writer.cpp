/* video_writer.cpp */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "ros_compat.h"

// #include "std_srvs/Empty.h"

cv::VideoWriter *vw;
int frameCount = 0;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try {
    ++frameCount;
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(3);

    vw->write(cv_bridge::toCvShare(msg, "bgr8")->image);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  //  TODO  - Have to do some kind of persistant file index instead of relying on time which is only updated when
  //  connected to the internet!
  // return;
  // struct timespec initial_time, current_time;
  // clock_gettime(CLOCK_REALTIME, &initial_time);
  vw = new cv::VideoWriter("/home/boo/proj/roscol/captures/video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                           cv::Size(1920, 1080), true);

  ros::init(argc, argv, "video_writer");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("jetcam/image", 1, imageCallback);

  cv::namedWindow("view");
  while (ros::ok() && frameCount < 100) {
    ros::spinOnce();
  }

  cv::destroyWindow("view");

  vw->release();
  delete vw;

  cv::VideoCapture vc("/home/boo/proj/roscol/captures/video.avi");

  int count = 0;
  cv::Mat img;
  char buf[256];
  while (vc.read(img)) {
    sprintf(buf, "/home/boo/proj/roscol/captures/video/img%s%s%i", count < 100 ? "0" : "", count < 10 ? "0" : "",
            count);
    cv::imwrite(buf, img);
    ++count;
  }

  vc.release();
  ROS_INFO("Wrote %i images", count);

  return 0;
}
