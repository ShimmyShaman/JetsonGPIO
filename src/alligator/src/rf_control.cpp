/* rf_control.cpp */

// #include <errno.h>
// #include <fcntl.h>
// #include <poll.h>
// #include <pthread.h>
// #include <stdio.h>
// // #include <stdint.h> ?? TODO
// #include <stdlib.h>
// #include <string.h>
// #include <time.h>
#include <dirent.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
// #include "ros_compat.h"

#include <image_transport/image_transport.h>

#include <opencv2/videoio.hpp>

#include "JetsonNanoRadiohead/RH_NRF24.h"
#include "JetsonNanoRadiohead/RHutil/JetsonNano_gpio.h"
#include "std_srvs/Empty.h"
// #include "RH_NRF24.h"

#define GPIO_LIFT 149
#define GPIO_ROTATER 200
#define GPIO_APWR 168
#define GPIO_ADIR 51
#define GPIO_BPWR 12
#define GPIO_BDIR 76

// SECTION
// This section must be kept in sync between the Nano-side Collector App and the
// jsapp
#define SIG_LEN 3
uint8_t signature[] = {113, 176, 87};

#define PID_COUNTER_EMAX 250
#define CAPTURE_IMAGE_SEQUENCE_DURATION 30

enum CommunicationType {
  COMMUNICATION_NULL = 0,
  COMMUNICATION_CONNECT_AUTO,
  COMMUNICATION_CONNECT_RCOVER,
  COMMUNICATION_SPEED_SET_1,
  COMMUNICATION_SPEED_SET_2,
  COMMUNICATION_SPEED_SET_3,
  COMMUNICATION_SPEED_SET_4,
  COMMUNICATION_SPEED_SET_5,
  COMMUNICATION_RCMOVE,
  COMMUNICATION_RCIDLE,
  COMMUNICATION_CAPTURE_IMAGE,
  COMMUNICATION_CAPTURE_IMAGE_SEQUENCE,
  COMMUNICATION_STOP_CAPTURE_SEQUENCE,
  COMMUNICATION_NANO_SHUTDOWN = 127,
};

enum ControllerModeType {
  CONTROLLER_MODE_NULL = 0,
  CONTROLLER_MODE_AUTONOMOUS = 2,
  CONTROLLER_MODE_RCOVERRIDE = 5,
};
// END-SECTION

#define MOTOR_SET_SPEED_1 600 // 1
#define MOTOR_SET_SPEED_2 800 // 6
#define MOTOR_SET_SPEED_3 900 // 36
#define MOTOR_SET_SPEED_4 950 //
#define MOTOR_SET_SPEED_5 1000

typedef struct MotorThreadData {
  pthread_t tid;
  unsigned int drive_gpio, dir_gpio;
  // int *maxSpeed;
  float power; // (0->1)
  int dir;
  bool doExit;
  bool doPause;
  bool finished;
  const char *name;
} MotorThreadData;

MotorThreadData motorA, motorB;

// Singleton instance of the radio driver

RH_NRF24 nrf24(13, 19);  // For the Nvidia Jetson Nano (gpio pins 13, 19 are J41
                         // 22, 24 respectively, [ChipEnable, SlaveSelect])

uint32_t captureTransferDelay = 0;
const char *CAPTURE_DIR = "/home/boo/proj/roscol/captures";
const char *USB_CAPTURE_FOLDER = "/media/boo/transcend/captures";

// Speed between 0 and 1
int maxSpeed;

// The mode (RCOverride or Autonomous)
enum ControllerModeType cmode;

bool shutdown_requested = false;
ros::ServiceClient save_image_client;
int captureImageState;
std_srvs::Empty empty_msg;

void *motorThread(void *arg)
{
  MotorThreadData *m = (MotorThreadData *)arg;
  ROS_INFO("%s thread opened!", m->name);

  int setDir = 99999;

  // DEBUG
  int dn = 0;
  // DEBUG

  while (!m->doExit) {
    if (m->doPause) {
      ensure_set_value(m->drive_gpio, 0);
      usleep(1000);
      continue;
    }
    else {
      // Stop this from continuing on if main thread collapses
    }

    int mex = (int)(m->power * maxSpeed);

    // DEBUG
    // ++dn;
    // if (dn % 1000 == 0) {
    //   printf("%s mex=%i\n", m->name, mex);
    //   dn -= 1000;
    // }
    // DEBUG

    if (mex) {
      if (m->dir != setDir) {
        setDir = m->dir;
        ensure_set_value(m->dir_gpio, setDir);
      }

      ensure_set_value(m->drive_gpio, 1);
      usleep(mex);
      if (mex < 1000) {
        ensure_set_value(m->drive_gpio, 0);
        usleep(1000 - mex);
      }
      else {
        continue;
      }
    }
    else {
      ensure_set_value(m->drive_gpio, 0);
      usleep(1000);
    }
  }

  ensure_set_value(m->drive_gpio, 0);
  m->finished = true;
  ROS_INFO("%s thread-closed", m->name);
  pthread_exit(NULL);
}

int cp(const char *source, const char *destination)
{
  int input, output;
  if ((input = open(source, O_RDONLY)) == -1) {
    return -1;
  }
  if ((output = creat(destination, 0660)) == -1) {
    close(input);
    return -2;
  }

  // sendfile will work with non-socket output (i.e. regular file) on Linux 2.6.33+
  off_t bytesCopied = 0;
  struct stat fileinfo = {0};
  fstat(input, &fileinfo);
  int result = sendfile(output, input, &bytesCopied, fileinfo.st_size);

  close(input);
  close(output);

  return result;
}

void changeMode(enum ControllerModeType mode)
{
  if (mode == CONTROLLER_MODE_RCOVERRIDE && cmode == CONTROLLER_MODE_AUTONOMOUS) {
    maxSpeed = 0;
  }

  cmode = mode;

  if (mode == CONTROLLER_MODE_AUTONOMOUS) {
    maxSpeed = 0;
  }
}

void jetcamImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // ROS_INFO("jetcamImageCallback");
  if (captureImageState) {
    --captureImageState;
    save_image_client.call(empty_msg);

    if (!captureImageState) {
      // Delay a capture transfer (latest image to the usb) for 2 frames
      captureTransferDelay = 2;
    }
  }
}

void attemptLatestImageTransferToUSB(void)
{
  struct stat sb;
  if (stat(USB_CAPTURE_FOLDER, &sb) != 0 || !S_ISDIR(sb.st_mode)) {
    ROS_INFO("USB DRIVE NOT FOUND");
    return;
  }
  ROS_INFO("USB DRIVE FOUND");

  char nb[64], ovb[512], mvb[512];
  int latest_img_nb = -1;
  nb[4] = '\0';

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(CAPTURE_DIR)) != NULL) {
    // Obtain all children in the directory
    while ((ent = readdir(dir)) != NULL) {
      int len = strlen(ent->d_name);
      if (len >= 8 && !strncmp(".jpg", ent->d_name + len - 4, 4)) {

        memcpy(nb, ent->d_name + len - (4 + 4), sizeof(char) * 4);
        int n = atoi(nb);

        if (n > latest_img_nb) {
          ROS_INFO("Latest Img Nb is : %i", n);
          latest_img_nb = n;
          sprintf(ovb, "%s/%s", CAPTURE_DIR, ent->d_name);
          // sprintf(cmd, "cp %s %s/%s", ovb, USB_CAPTURE_FOLDER, ent->d_name);
          sprintf(mvb, "%s/%s", USB_CAPTURE_FOLDER, ent->d_name);
        }
      }
      else {
        // ROS_INFO("Ignoring file entry '%s' as not jpg", ent->d_name);
      }
    }

    ROS_INFO("Begin USB Img Transfer: %s", ovb);
    if (latest_img_nb >= 0) {
      int result = cp(ovb, mvb);
      if (result < 0) {
        ROS_INFO("Error(%i) copying jpg: %s >> %s", result, ovb, mvb);
      }
      else {
        ROS_INFO("Copied file '%s' to USB: %s (%i bytes)", ovb, mvb, result);
      }
    }
    closedir(dir);
  }
  else {
    /* could not open directory */
    ROS_INFO("could not open directory '%s'", CAPTURE_DIR);
  }
}

void transferSessionCaptures()
{
  char buf[512], ovb[512], mvb[512];
  // sprintf(buf, "%s/%i_%i%s%i", CAPTURE_DIR, tm_struct->tm_mday, tm_struct->tm_hour, (tm_struct->tm_min / 10) ? "" :
  // "0", tm_struct->tm_min);
  // mkdir(buf, 0777);

  int session_idx = 1;
  struct stat sb;

  for (;; ++session_idx) {
    sprintf(buf, "%s/s%i", CAPTURE_DIR, session_idx);
    if (stat(buf, &sb) != 0 || !S_ISDIR(sb.st_mode))
      break;
  }
  bool dir_created = false;

  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir(CAPTURE_DIR)) != NULL) {
    // Obtain all children in the directory
    while ((ent = readdir(dir)) != NULL) {
      int len = strlen(ent->d_name);
      if (len >= 5 && !strncmp(".jpg", ent->d_name + len - 4, 4)) {
        sprintf(ovb, "%s/%s", CAPTURE_DIR, ent->d_name);
        sprintf(mvb, "%s/%s", buf, ent->d_name);

        if (!dir_created) {
          // Create it before moving files there
          dir_created = true;
          if (mkdir(buf, 0777)) {
            ROS_INFO("Could not create directory for session captures, aborting");
            closedir(dir);
            return;
          }
        }

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
    ROS_INFO("could not open directory '%s'", CAPTURE_DIR);
  }
}

bool setup(ros::NodeHandle &nh)
{
  cmode = CONTROLLER_MODE_AUTONOMOUS;
  maxSpeed = MOTOR_SET_SPEED_1;

  // Image Saver
  save_image_client = nh.serviceClient<std_srvs::Empty>("/image_view/save");
  captureImageState = 0;

  ensure_export(13);
  ensure_export(19);

  // Begin RF Communication
  // Serial.begin(9600);
  ROS_INFO("initializing...");
  bool scs;
  int r = 100;
  while (1) {
    scs = nrf24.init();
    if (scs) {
      ROS_INFO("nrf24.init() succeeded");
      break;
    }
    ROS_INFO("nrf24.init() failed");
    --r;
    if (r < 0) {
      ROS_INFO("nrf24.init() abandoned");
      return false;
    }
    usleep(1000000);
  }

  // Defaults after init are 2.402 GHz (channel 6), 2Mbps, 0dBm
  if (!nrf24.setChannel(6)) {
    ROS_INFO("nrf24.setChannel() failed");
    return false;
  }

  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) {
    ROS_INFO("nrf24.setRF() failed");
    return false;
  }

  ROS_INFO("nrf24 initialization succeeded");

  ensure_export(GPIO_LIFT);
  ensure_set_dir(GPIO_LIFT, OUTPUT);
  ensure_export(GPIO_ROTATER);
  ensure_set_dir(GPIO_ROTATER, OUTPUT);
  ensure_export(GPIO_APWR);
  ensure_set_dir(GPIO_APWR, OUTPUT);
  ensure_export(GPIO_ADIR);
  ensure_set_dir(GPIO_ADIR, OUTPUT);
  ensure_export(GPIO_BDIR);
  ensure_set_dir(GPIO_BDIR, OUTPUT);
  ensure_export(GPIO_BPWR);
  ensure_set_dir(GPIO_BPWR, OUTPUT);

  // Begin Motor Threads
  int rc;

  motorA.doExit = false;
  motorA.doPause = false;
  motorA.finished = false;
  motorA.drive_gpio = GPIO_APWR;
  motorA.dir_gpio = GPIO_ADIR;
  motorA.dir = 0;
  motorA.power = 0.f;
  motorA.name = "MotorA";
  if ((rc = pthread_create(&motorA.tid, NULL, motorThread, &motorA))) {
    fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
    return EXIT_FAILURE;
  }

  motorB.doExit = false;
  motorB.doPause = false;
  motorB.finished = false;
  motorB.drive_gpio = GPIO_BPWR;
  motorB.dir_gpio = GPIO_BDIR;
  motorB.dir = 0;
  motorB.power = 0.f;
  motorB.name = "MotorB";
  if ((rc = pthread_create(&motorB.tid, NULL, motorThread, &motorB))) {
    fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
    return EXIT_FAILURE;
  }

  return true;
}

void loop()
{
  uint64_t time = millis();

  //   // nrf24.waitPacketSent();
  //   // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // ROS_INFO("waitAvailableTimeout");
  if (nrf24.available()) {
    // Should be a reply message for us now
    if (nrf24.recv(buf, &len)) {
      // printf("got reply: ");
      // ROS_INFO((char *)buf);

      bool verified = len >= SIG_LEN;
      if (verified) {
        for (int i = 0; i < SIG_LEN; ++i) {
          if (buf[i] != signature[i]) {
            verified = false;
            break;
          }
        }
      }
      if (verified) {
        enum CommunicationType ct = (enum CommunicationType)buf[SIG_LEN];

        bool replyComConfirm = true;
        __uint8_t confirmPacketUID = buf[SIG_LEN + 1];
        switch (ct) {
          case COMMUNICATION_CONNECT_AUTO:
            changeMode(CONTROLLER_MODE_AUTONOMOUS);
            ROS_INFO("CONNECT_AUTO");
            break;
          case COMMUNICATION_CONNECT_RCOVER:
            changeMode(CONTROLLER_MODE_RCOVERRIDE);
            ROS_INFO("CONNECT_RCOVER");
            break;
          case COMMUNICATION_RCIDLE: {
            motorA.power = 0;
            motorB.power = 0;
          } break;
          case COMMUNICATION_RCMOVE: {
            changeMode(CONTROLLER_MODE_RCOVERRIDE);

            // DEBUG
            maxSpeed = MOTOR_SET_SPEED_5;
            float jx = (float)buf[SIG_LEN + 1], jy = (float)buf[SIG_LEN + 2];
            // printf("[0]:%.1f  [1]:%.1f\n", jx, jy);
            // DEBUG

            const float THRES = 4.f, MID = 63.5;
            if (jx > MID - THRES && jx < MID + THRES && jy > MID - THRES && jy < MID + THRES) {
              motorA.power = 0;
              motorB.power = 0;
            }
            else {
              motorA.dir = jx > MID ? 1 : 0;
              motorB.dir = jy > MID ? 0 : 1;

              motorA.power = abs((jx - MID) * 1.05f) / MID;
              motorB.power = abs((jy - MID) * 1.05f) / MID;
            }

            // // TODO check this is okay with 127 no movement
            // motorA.power = ((float)buf[SIG_LEN + 1] - 127.f) / 128.f;
            // if (motorA.power < 0) {
            //   motorA.dir = 1;
            //   motorA.power = -motorA.power;
            // }
            // else {
            //   motorA.dir = 0;
            // }
            // motorB.power = 1.f;//((float)buf[SIG_LEN + 2] - 127.f) / 128.f;
            // if (motorB.power < 0) {
            //   motorB.dir = 1;
            //   motorB.power = -motorB.power;
            // }
            // else {
            //   motorB.dir = 0;
            // }

            // TODO -- Not sending a packet uid to verify -- make it cleaner
            // replyComConfirm = false;
            confirmPacketUID = PID_COUNTER_EMAX;
            printf("CONNECT_RCMOVE: A[%i, %.3f] B[%i, %.3f]\n", motorA.dir, motorA.power, motorB.dir, motorB.power);
          } break;
          case COMMUNICATION_NANO_SHUTDOWN:
            ROS_INFO("Nano Shutdown");
            shutdown_requested = true;
            break;
          case COMMUNICATION_SPEED_SET_1:
            maxSpeed = MOTOR_SET_SPEED_1;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_1");
            break;
          case COMMUNICATION_SPEED_SET_2:
            maxSpeed = MOTOR_SET_SPEED_2;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_2");
            break;
          case COMMUNICATION_SPEED_SET_3:
            maxSpeed = MOTOR_SET_SPEED_3;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_3");
            break;
          case COMMUNICATION_SPEED_SET_4:
            maxSpeed = MOTOR_SET_SPEED_4;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_4");
            break;
          case COMMUNICATION_SPEED_SET_5:
            maxSpeed = MOTOR_SET_SPEED_5;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_5");
            break;
          case COMMUNICATION_CAPTURE_IMAGE:
            if (!captureImageState) {
              captureImageState = 1;
              // captureTransferDelay = 4;
              ROS_INFO("Beginning Single Image Capture");
            }
            break;
          case COMMUNICATION_CAPTURE_IMAGE_SEQUENCE:
            ROS_INFO("Beginning Image Capture Sequence");
            captureImageState = 4 * CAPTURE_IMAGE_SEQUENCE_DURATION;
            break;
          case COMMUNICATION_STOP_CAPTURE_SEQUENCE:
            ROS_INFO("Aborting Image Capture Sequence");
            captureImageState = 0;
            break;
          default:
            printf("Unrecognised Verified Communication:'%s'\n", buf + SIG_LEN);
            break;
        }
        if (replyComConfirm) {
          // Send connection confirmation
          uint8_t data[SIG_LEN + 1];
          for (int i = 0; i < SIG_LEN; ++i) {
            data[i] = signature[i];
          }

          // In Packets Expecting Confirmation PacketUID is in the +1 position
          data[SIG_LEN] = confirmPacketUID;

          nrf24.send(data, sizeof(data));
          nrf24.waitPacketSent();

          // ROS_INFO("Sent Connectivity Response");
        }
      }
    }
    else {
      ROS_INFO("recv failed");
    }
  }
  else {
    // ROS_INFO("No reply, is nrf24_server running?");
  }

  if (captureTransferDelay) {
    --captureTransferDelay;
    if (!captureTransferDelay) {
      attemptLatestImageTransferToUSB();
    }
  }
}

void cleanup()
{
  // End Motor Threads
  motorA.doExit = motorB.doExit = true;

  // -- Do other stuff while waiting for threads to exit on their own
  // Move any captures into a session folder
  transferSessionCaptures();

  // Ensure Motor Threads ended
  int r = 0;
  while ((!motorA.finished || !motorB.finished) && r < 4000) {
    usleep(500);
    ++r;
  }

  // Check thread-hanging timeout conditions
  if (motorA.finished) {
    ROS_INFO("Trouble shutting down MotorA-Thread");
    // TODO pthread_abort handling
  }
  if (motorB.finished) {
    ROS_INFO("Trouble shutting down MotorB-Thread");
    // TODO pthread_abort handling
  }

  // TODO -- unexport??
  gpio_unexport(13);
  gpio_unexport(19);
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);               // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

int main(int argc, char **argv)
{
  ROS_INFO("rf_control Begun");

  // ROS_INFO("Press 'q' to exit");
  ros::init(argc, argv, "rf_control");
  ros::NodeHandle nh;

  if (!setup(nh)) {
    cleanup();

    ROS_INFO("rf_control Failed Setup");
    return -1;
  }

  // captureImageState = 22;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("jetcam/image", 1, jetcamImageCallback);

  ros::Rate loop_rate(8);
  while (ros::ok() && !shutdown_requested) {
    // Call your non-blocking input function
    // int c = getch();
    // if (c == 'q') {
    //   break;
    // }

    loop();
    loop_rate.sleep();
    ros::spinOnce();
    // ROS_INFO("captureImageState=%i", captureImageState);
  }

  // ROS_INFO("cleanup");
  cleanup();

  if (shutdown_requested) {
    system("shutdown -P now");
  }

  ROS_INFO("rf_control Exited");
  usleep(1000);
  return 0;
}

// int main(int argc, char **argv)
// {
//   return 0;
//   ROS_INFO("Entered App...");
//   usleep(1000000 * 8);

//   // ensure_export(GPIO_ADIR);
//   // ensure_export(GPIO_APWR);
//   // ensure_export(GPIO_BDIR);
//   // ensure_export(GPIO_BPWR);

//   // ensure_set_dir(GPIO_APWR, OUTPUT);
//   // ensure_set_dir(GPIO_ADIR, OUTPUT);
//   // ensure_set_dir(GPIO_BDIR, OUTPUT);
//   // ensure_set_dir(GPIO_BPWR, OUTPUT);
//   // ensure_set_value(GPIO_ADIR, 1);
//   // ensure_set_value(GPIO_BDIR, 1);

//   // for (int m = 1000; m >= 900; m -= 5)
//   // {
//   //   // printf("trying speed %i / %i\n", 1 + m, 1000 - m);
//   //   // for (int r = 0; r < 3000; ++r)
//   //   // {
//   //   //   ensure_set_value(GPIO_APWR, 1);
//   //   //   ensure_set_value(GPIO_BPWR, 1);
//   //   //   usleep(1 + m);
//   //   //   ensure_set_value(GPIO_APWR, 0);
//   //   //   ensure_set_value(GPIO_BPWR, 0);
//   //   //   usleep(1000 - m);
//   //   // }

//   //   // ensure_set_value(GPIO_APWR, 1);
//   //   ensure_set_value(GPIO_BPWR, 1);
//   //   usleep(3000000);
//   //   ensure_set_value(GPIO_APWR, 0);
//   //   ensure_set_value(GPIO_BPWR, 0);
//   //   usleep(3000000);
//   // }
//   // ensure_set_value(GPIO_APWR, 0);
//   // ensure_set_value(GPIO_BPWR, 0);

//   // return 0;

//   setup();

//   // unsigned long count = millis();
//   while (1)
//     loop();

//   cleanup();

//   // rungpio5secs(149);
//   // rungpio5secs(200);
//   // // rungpio5secs(168);
//   // // rungpio5secs(38);

//   // // ensure_set_dir(149, 1);
//   // // ensure_set_dir(168, 1);

//   // for (int m = 100; m >= 0; m -= 10)
//   // {
//   //   printf("trying speed %i / %i\n", 1 + m, 1000 - m);
//   //   for (int r = 0; r < 7000; ++r)
//   //   {
//   //     ensure_set_value(168, 1);
//   //     usleep(1 + m);
//   //     ensure_set_value(168, 0);
//   //     usleep(1000 - m);
//   //   }
//   // }
//   // ensure_set_value(168, 0);

//   // ensure_set_value(149, 1);
//   // ensure_set_value(168, 1);

//   delay(1000);

//   ROS_INFO("exiting...");
//   exit(0);
// }
