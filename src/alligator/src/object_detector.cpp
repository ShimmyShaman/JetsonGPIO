#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "ros_compat.h"

void SaveImageAsPPM(const sensor_msgs::Image *msg, const char *filename)
{
  if (msg->encoding != "bgr8")
  {
    ROS_INFO("SaveImageAsPPM: encoding=%s NOT bgr8", msg->encoding.c_str());
    return; // Can only handle the rgb8 encoding
  }

  ROS_INFO("opening file: %s", filename);
  FILE *file = fopen(filename, "w");
  if (!file)
  {
    ROS_INFO("Could not open file");
    return;
  }

  ROS_INFO("a");
  fprintf(file, "P3\n");
  ROS_INFO("b");
  fprintf(file, "%i %i\n", msg->width, msg->height);
  fprintf(file, "255\n");

  ROS_INFO("c");
  for (uint32_t y = 0; y < msg->height; y++)
  {
    for (uint32_t x = 0; x < msg->width; x++)
    {
      // Get indices for the pixel components
      uint32_t blueByteIdx = y * msg->step + 3 * x;
      uint32_t greenByteIdx = blueByteIdx + 1;
      uint32_t redByteIdx = blueByteIdx + 2;

      fprintf(file, "%i %i %i ",
              msg->data[blueByteIdx],
              msg->data[greenByteIdx],
              msg->data[redByteIdx]);
    }
    fprintf(file, "\n");
  }

  ROS_INFO("closing file");

  fclose(file);
}

int saves = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
//   try
//   {
    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    // cv::waitKey(30);
    ROS_INFO("Received image");
//   }
//   catch (cv_bridge::Exception& e)
//   {
    // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
if(!saves) {
    SaveImageAsPPM(msg.get(), "/home/boo/proj/roscol/nmg.ppm");
}
++saves;
}

int main(int argc, char **argv)
{
    ROS_INFO("Begun");
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
//   cv::namedWindow("view");

    ROS_INFO("Subscribing");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("jetcam/image", 1, imageCallback);
  ros::spin();
//   cv::destroyWindow("view");
}