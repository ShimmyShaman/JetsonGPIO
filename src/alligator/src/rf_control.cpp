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
// #include <math.h>
#include <dirent.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/sendfile.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
// #include "ros_compat.h"

#include <image_transport/image_transport.h>

#include <opencv2/videoio.hpp>

#include "JetsonNanoRadiohead/RH_NRF24.h"
// #include "JetsonNanoRadiohead/RHutil/JetsonNano_gpio.h"
#include "std_srvs/Empty.h"

#include "JetsonNanoRadiohead/RHutil/JetsonNano_gpio.h"
#include <JetsonGPIO.h>

#include "std_srvs/Empty.h"

#define GPIO_LIFT 29    /*149*/
#define GPIO_ROTATER 31 /*200*/
#define GPIO_APWR 32    /*168*/
#define GPIO_ADIR 36    /*51*/
#define GPIO_BPWR 33    /*38*/
#define GPIO_BDIR 35    /*76*/
#define GPIO_UNUSED 37  /*12*/

#define GPIO_AENC 7  /*216*/
#define GPIO_BENC 11 /*50*/

#define GPIO_NRF_CE 22 /*13*/
#define GPIO_NRF_SS 24 /*19*/

// COMMON SECTION
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
// Speed in mm per second
#define MOTOR_SET_SPEED_1 250
#define MOTOR_SET_SPEED_2 500
#define MOTOR_SET_SPEED_3 750
#define MOTOR_SET_SPEED_4 1000
#define MOTOR_SET_SPEED_5 2000

typedef struct MotorThreadData {
  pthread_t tid;
  unsigned int drive_channel, dir_channel;
  float target_speed, est_speed; // in mm per second -- running average
  int dir;
  std::atomic<unsigned int> enc_register;
  bool doExit;
  bool doPause;
  bool finished;
  const char *name;
} MotorThreadData;

MotorThreadData motorA, motorB;

// Singleton instance of the radio driver

RH_NRF24 nrf24(GPIO_NRF_CE, GPIO_NRF_SS); // For the Nvidia Jetson Nano (gpio pins 13, 19 are J41
                                          // 22, 24 respectively, [ChipEnable, SlaveSelect])

uint32_t captureTransferDelay = 0;
const char *CAPTURE_DIR = "/home/boo/proj/roscol/captures";
const char *USB_CAPTURE_FOLDER = "/media/boo/transcend/captures";

// Speed in mm per second
int max_speed;

// The mode (RCOverride or Autonomous)
enum ControllerModeType current_mode;

bool shutdown_requested = false;
ros::ServiceClient save_image_client;
int captureImageState;
std_srvs::Empty empty_msg;

void motor_encoding_signal_callback(int channel)
{
  switch (channel) {
    case GPIO_AENC:
      ++motorA.enc_register;
      break;
    case GPIO_BENC:
      ++motorB.enc_register;
      break;
  }
}

void *motorThread(void *arg)
{
  MotorThreadData *m = (MotorThreadData *)arg;
  ROS_INFO("%s thread opened!", m->name);

  int setDir = 99999;

  // DEBUG
  int dn = 0;
  // DEBUG

  unsigned int prev_enc_sig = 0U;
  int power = 0;
  const int MAX_POWER = 998;

  const unsigned int BIN_SIZE = 500;
  unsigned int sigbin[BIN_SIZE], si = 0, sigtot = 0, diff;
  memset(sigbin, 0, sizeof(sigbin));

  while (!m->doExit) {
    // m->target_speed = 300.f;

    if (m->doPause) {
      GPIO::output(m->drive_channel, 0);
      // ensure_set_value(m->drive_channel, 0);
      usleep(1000);
      continue;
    }
    else {
      // Stop this from continuing on if main thread collapses
    }

    // Update the wheel rotation
    ++si;
    if (si >= BIN_SIZE)
      si -= BIN_SIZE;

    diff = m->enc_register - prev_enc_sig;
    prev_enc_sig = m->enc_register;
    sigtot += diff - sigbin[si];
    sigbin[si] = diff;

    // -- 225 encoder signals per turn - 30cm per turn : 1.2 mm per signal
    m->est_speed = 1000.f / BIN_SIZE * 1.2f * sigtot;

    if (m->dir != setDir) {
      // Wrong way
      setDir = m->dir;
      GPIO::output(m->dir_channel, setDir);
      power = 0;
      usleep(999);
      continue;
    }
    if (m->target_speed == 0.f) {
      power = 0;
      usleep(999);
      continue;
    }

    // power = 100;

    // else {

    // if (si % 40 == 0) {
    if (m->est_speed < m->target_speed) {
      if (power < MAX_POWER)
        ++power;
    }
    else {
      if (power > 0)
        --power;
    }
    // }
    //   // Adjust speed
    //   float ratio;
    //   if (m->est_speed)
    //     ratio = m->target_speed / m->est_speed;
    //   else
    //     ratio = 2;

    //   float sign = 1;
    //   if (m->est_speed > m->target_speed) {
    //     ratio = 1.f / ratio;
    //     sign = -1;
    //   }

    //   float pwr_chg = sign * 1; //((ratio - 1.f) * 5 + pow(ratio, 7));
    //   power += (int)pwr_chg;
    //   if (power < 0)
    //     power = 0;
    //   else if (power > MAX_POWER)
    //     power = MAX_POWER;

    // // DEBUG
    // if (si == 0 && !strcmp(m->name, "MotorA")) {
    //   ROS_INFO("Speed: motorA:[%i]%.2f(%.2f) motorB:%.2f(%.2f)", power, motorA.est_speed, motorA.target_speed,
    //            motorB.est_speed, motorB.target_speed);
    // }
    // if (!strcmp(m->name, "MotorB") && si == 9) {
    //   printf("dir:%i\n", m->dir_channel);
    // }
    // // DEBUG

    if (power) {
      GPIO::output(m->drive_channel, 1);
      usleep(power);
    }
    GPIO::output(m->drive_channel, 0);
    if (power < MAX_POWER)
      usleep(MAX_POWER - power);
  }

  GPIO::output(m->drive_channel, 0);
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
  if (current_mode == CONTROLLER_MODE_AUTONOMOUS && mode == CONTROLLER_MODE_RCOVERRIDE) {
    motorA.target_speed = 0;
    motorB.target_speed = 0;
  }

  if (mode == CONTROLLER_MODE_AUTONOMOUS) {
    motorA.target_speed = MOTOR_SET_SPEED_5;
    motorA.dir = 1;
    motorB.target_speed = MOTOR_SET_SPEED_5;
    motorB.dir = 0;
  }

  current_mode = mode;
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
  current_mode = CONTROLLER_MODE_AUTONOMOUS;
  max_speed = MOTOR_SET_SPEED_5;

  // Image Saver
  save_image_client = nh.serviceClient<std_srvs::Empty>("/image_view/save");
  captureImageState = 0;

  GPIO::setmode(GPIO::BOARD);

  // GPIO::setup(13, GPIO::Directions::OUT);
  // GPIO::setup(19, GPIO::Directions::OUT);
  ensure_export(GPIO_NRF_CE);
  ensure_export(GPIO_NRF_SS);

  // Begin RF Communication
  // Serial.begin(9600);
  ROS_INFO("initializing...");
  bool scs;
  int r = 100;
  while (true) {
    if (!ros::ok() || shutdown_requested)
      return false;

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

  GPIO::setup(GPIO_LIFT, GPIO::Directions::OUT);
  GPIO::setup(GPIO_ROTATER, GPIO::Directions::OUT);
  GPIO::setup(GPIO_APWR, GPIO::Directions::OUT);
  GPIO::setup(GPIO_ADIR, GPIO::Directions::OUT);
  GPIO::setup(GPIO_BDIR, GPIO::Directions::OUT);
  GPIO::setup(GPIO_BPWR, GPIO::Directions::OUT);

  GPIO::setup(GPIO_AENC, GPIO::Directions::IN);
  GPIO::setup(GPIO_BENC, GPIO::Directions::IN);

  // Begin Motor Threads
  int rc;

  memset(&motorA, 0, sizeof(MotorThreadData));
  // motorA.doExit = false;
  // motorA.doPause = false;
  // motorA.finished = false;
  motorA.drive_channel = GPIO_APWR;
  motorA.dir_channel = GPIO_ADIR;
  // motorA.dir = 0;
  // motorA.enc_register = 0;
  // motorA.target_speed = 0.f;
  // motorA.est_speed = 0.f;
  motorA.name = "MotorA";
  if ((rc = pthread_create(&motorA.tid, NULL, motorThread, &motorA))) {
    fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
    return EXIT_FAILURE;
  }

  memset(&motorB, 0, sizeof(MotorThreadData));
  // motorB.doExit = false;
  // motorB.doPause = false;
  // motorB.finished = false;
  motorB.drive_channel = GPIO_BPWR;
  motorB.dir_channel = GPIO_BDIR;
  // motorB.dir = 0;
  // motorB.enc_register = 0;
  // motorB.target_speed = 0.f;
  // motorB.est_speed = 0.f;
  motorB.name = "MotorB";
  if ((rc = pthread_create(&motorB.tid, NULL, motorThread, &motorB))) {
    fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
    return EXIT_FAILURE;
  }

  GPIO::add_event_detect(GPIO_AENC, GPIO::Edge::RISING, motor_encoding_signal_callback);
  GPIO::add_event_detect(GPIO_BENC, GPIO::Edge::RISING, motor_encoding_signal_callback);

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
            motorA.target_speed = 0;
            motorB.target_speed = 0;
          } break;
          case COMMUNICATION_RCMOVE: {
            changeMode(CONTROLLER_MODE_RCOVERRIDE);

            float jx = (float)buf[SIG_LEN + 1], jy = (float)buf[SIG_LEN + 2];

            // DEBUG
            // max_speed = MOTOR_SET_SPEED_5;
            // printf("[0]:%.1f  [1]:%.1f\n", jx, jy);
            // DEBUG

            const float THRES = 4.f, MID = 63.5;
            if (jx > MID - THRES && jx < MID + THRES && jy > MID - THRES && jy < MID + THRES) {
              motorA.target_speed = 0;
              motorB.target_speed = 0;
            }
            else {
              // Because the motors are orientated in opposite directions
              //  (one pointed towards left, other right)
              motorA.dir = jx > MID ? 1 : 0;
              motorB.dir = jy > MID ? 0 : 1;

              motorA.target_speed = abs((jx - MID) * 1.05f) / MID * max_speed;
              if (motorA.target_speed > max_speed)
                motorA.target_speed = max_speed;
              motorB.target_speed = abs((jy - MID) * 1.05f) / MID * max_speed;
              if (motorB.target_speed > max_speed)
                motorB.target_speed = max_speed;
            }

            // TODO -- Not sending a packet uid to verify -- make it cleaner
            // replyComConfirm = false;
            confirmPacketUID = PID_COUNTER_EMAX;
            printf("CONNECT_RCMOVE: A[%i, %.3f] B[%i, %.3f]\n", motorA.dir, motorA.target_speed, motorB.dir,
                   motorB.target_speed);
          } break;
          case COMMUNICATION_NANO_SHUTDOWN:
            ROS_INFO("Nano Shutdown");
            shutdown_requested = true;
            break;
          case COMMUNICATION_SPEED_SET_1:
            max_speed = MOTOR_SET_SPEED_1;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_1");
            break;
          case COMMUNICATION_SPEED_SET_2:
            max_speed = MOTOR_SET_SPEED_2;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_2");
            break;
          case COMMUNICATION_SPEED_SET_3:
            max_speed = MOTOR_SET_SPEED_3;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_3");
            break;
          case COMMUNICATION_SPEED_SET_4:
            max_speed = MOTOR_SET_SPEED_4;
            ROS_INFO("Speed Set to MOTOR_SET_SPEED_4");
            break;
          case COMMUNICATION_SPEED_SET_5:
            max_speed = MOTOR_SET_SPEED_5;
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
            if (!captureImageState) {
              captureImageState = 4 * CAPTURE_IMAGE_SEQUENCE_DURATION;
            }
            break;
          case COMMUNICATION_STOP_CAPTURE_SEQUENCE:
            ROS_INFO("Aborting Image Capture Sequence");
            captureImageState = 0;
            captureTransferDelay = 2;
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
          ROS_INFO("Sent Confirm %u", confirmPacketUID);

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
    // ROS_INFO("captureTransferDelay=%i", captureTransferDelay);
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

  // Unexport gpios
  GPIO::cleanup(GPIO_AENC);
  GPIO::cleanup(GPIO_BENC);

  GPIO::cleanup(GPIO_LIFT);
  GPIO::cleanup(GPIO_ROTATER);
  GPIO::cleanup(GPIO_APWR);
  GPIO::cleanup(GPIO_ADIR);
  GPIO::cleanup(GPIO_BDIR);
  GPIO::cleanup(GPIO_BPWR);

  // GPIO::cleanup(19);
  // GPIO::cleanup(13);
  gpio_unexport(GPIO_NRF_SS);
  gpio_unexport(GPIO_NRF_CE);
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