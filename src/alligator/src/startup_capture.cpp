#include <image_transport/image_transport.h>
#include <ros/ros.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "image_converter.h"
#include "ros_compat.h"

#include <jetson-utils/gstCamera.h>

#include "std_srvs/Empty.h"

bool initImageConverter(imageConverter **image_cvt)
{
  *image_cvt = new imageConverter();

  if (!image_cvt) {
    ROS_ERROR("failed to create imageConverter");
    return false;
  }
  return true;
}

bool createAndOpenCamera(gstCamera **pCamera, imageConverter *image_cvt)
{

  std::string resource_str = "csi://0";
  std::string codec_str = "";

  int video_width = 1920;  // video_options.width;
  int video_height = 1080; // video_options.height;
  if (resource_str.size() == 0) {
    ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
    return false;
  }

  ROS_INFO("opening video source: %s", resource_str.c_str());

  /*
   * open video source
   */
  gstCamera *camera = gstCamera::Create(video_width, video_height);

  if (!camera) {
    ROS_ERROR("failed to open video source");
    return false;
  }

  /*
   * start the camera streaming
  image_transport
   */
  if (!camera->Open()) {
    ROS_ERROR("failed to start streaming video source");
    delete camera;
    return false;
  }

  // Ensure correct CPU image size
  if (!image_cvt->Resize(camera->GetWidth(), camera->GetHeight(), imageConverter::ROSOutputFormat)) {
    ROS_ERROR("failed to resize camera image converter");

    camera->Close();
    delete camera;
    return false;
  }

  ROS_INFO("Camera Opened");
  *pCamera = camera;

  return true;
}

bool captureImage(gstCamera *camera, imageConverter *image_cvt, sensor_msgs::Image *img)
{

  imageConverter::PixelType *capture = NULL;
  if (!camera->Capture(&capture)) {
    ROS_ERROR("failed to capture image");
    return false;
  }

  // Convert the image to ROS output
  if (!image_cvt->Convert(*img, imageConverter::ROSOutputFormat, capture)) {
    ROS_ERROR("failed to convert image");
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
   TODO  - Have to do some kind of persistant file index instead of relying on time which is only updated when connected to the internet!
  return;
  struct timespec initial_time, current_time;
  clock_gettime(CLOCK_REALTIME, &initial_time);

  ros::init(argc, argv, "startup_capture");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber pub = it.advertise("jetcam/image", 1);

  sensor_msgs::ImagePtr msg = sensor_msgs::ImagePtr(new sensor_msgs::Image());

  // Image Saver
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/image_view/save");
  std_srvs::Empty svc;

  ros::Rate loop_rate(4);
  while (nh.ok()) {
    if (!captureImage(camera, image_cvt, msg.get())) {
      break;
    }

    pub.publish(msg);
    ROS_INFO("Image Published");
    ros::spinOnce();
    loop_rate.sleep();

    // Image Save Invocation
    if (client.call(svc)) {
      ROS_INFO("Sent image save request");
    }
    else {
      ROS_ERROR("Failed to call service image_saver::save");
      break;
    }
    // system("rosservice call /image_view/save");

    clock_gettime(CLOCK_REALTIME, &current_time);
    if (current_time.tv_sec - initial_time.tv_sec > 150) {
      ROS_INFO("Shutting down after 150 seconds");
      break;
    }
  }

  // Copy all captured images to a subfolder for the session
  // Create the subfolder
  time_t now = time(NULL);
  struct tm *tm_struct = localtime(&now);

  const char *CAPTURE_DIR = "/home/boo/proj/roscol/captures";
  char buf[512], ovb[512], mvb[512];
  sprintf(buf, "%s/%i_%i%s%i", CAPTURE_DIR, tm_struct->tm_mday, tm_struct->tm_hour, (tm_struct->tm_min / 10) ? "" : "0",
          tm_struct->tm_min);
  mkdir(buf, 0777);

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(CAPTURE_DIR)) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir(dir)) != NULL) {
      int len = strlen(ent->d_name);
      if (len >= 5 && !strncmp(".jpg", ent->d_name + len - 4, 4)) {
        sprintf(ovb, "%s/%s", CAPTURE_DIR, ent->d_name);
        sprintf(mvb, "%s/%s", buf, ent->d_name);

        if (rename(ovb, mvb)) {
          ROS_INFO("Error moving jpg: %s", ent->d_name);
        }
        else {
          ROS_INFO("Moved file '%s'", ent->d_name);
        }
      }
      else {
        // ROS_INFO("Ignoring file entry '%s' as not jpg", ent->d_name);
      }
    }
    closedir(dir);
  }
  else {
    /* could not open directory */
    ROS_INFO("could not open directory ~/proj/roscol/captures");
  }
  // system("mkdir /home/boo/proj/roscol/captures/1053");

  system("shutdown -P now");
  return 0;
}
