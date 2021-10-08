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
#include <time.h>
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

#define ENCODED_SPEED_MODE
#define TEST_NOISE_MODE

#define GPIO_ROTATER 29 /*200*/
#define GPIO_LIFT 31    /*149*/
#define GPIO_APWR 32    /*168*/
#define GPIO_ADIR 36    /*51*/
#define GPIO_BPWR 33    /*38*/
#define GPIO_BDIR 35    /*76*/
#define GPIO_UNUSED 37  /*12*/

#define GPIO_LENC 7  /*216*/
#define GPIO_RENC 11 /*50*/

#define GPIO_NRF_CE 13  // 22  /*13*/
#define GPIO_NRF_SS 19  // 24  /*19*/
#define GPIO_NRF_IRQ 26 /*20*/

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
#ifdef ENCODED_SPEED_MODE
// Speed in mm per second
#define MOTOR_SET_SPEED_1 360
#define MOTOR_SET_SPEED_2 480
#define MOTOR_SET_SPEED_3 600
#define MOTOR_SET_SPEED_4 720
#define MOTOR_SET_SPEED_5 1800
#define MOTOR_SPEED_MAX MOTOR_SET_SPEED_5
#else
#define MOTOR_SET_SPEED_1 40
#define MOTOR_SET_SPEED_2 55
#define MOTOR_SET_SPEED_3 70
#define MOTOR_SET_SPEED_4 85
#define MOTOR_SET_SPEED_5 100
#endif

typedef struct MotorThreadData {
  pthread_t tid;
  unsigned int drive_channel, dir_channel;
  int dir;
  std::atomic_bool doPause, doExit, finished;
  const char *name;
  std::atomic<float> power; // in mm per second -- running average

#ifdef ENCODED_SPEED_MODE
  std::atomic<float> est_speed; // in mm per second -- running average
  std::atomic_uint32_t enc_register;
  struct {
    std::atomic_int iterations_left;
    float est_speeds[250];
    std::atomic_bool completed;
  } debug;
#endif
} MotorThreadData;

MotorThreadData motorL, motorR;
bool rotator_active;

// Singleton instance of the radio driver

RH_NRF24 nrf24(GPIO_NRF_CE, GPIO_NRF_SS); // For the Nvidia Jetson Nano (gpio pins 13, 19 are J41
                                          // 22, 24 respectively, [ChipEnable, SlaveSelect])

uint32_t captureTransferDelay = 0;
const char *CAPTURE_DIR = "/home/boo/proj/roscol/captures";
const char *USB_CAPTURE_FOLDER = "/media/boo/transcend/captures";

// Speed in mm per second
std::atomic<int> max_speed;

// The mode (RCOverride or Autonomous)
enum ControllerModeType current_mode;
uint64_t last_com_time;

bool shutdown_requested = false;
ros::ServiceClient save_image_client;
int captureImageState;
std_srvs::Empty empty_msg;

#ifdef ENCODED_SPEED_MODE
void motor_encoding_signal_callback(int channel)
{
  switch (channel) {
    case GPIO_LENC:
      ++motorL.enc_register;
      break;
    case GPIO_RENC:
      ++motorR.enc_register;
      break;
  }
}
#endif

void *motorThread(void *arg)
{
  MotorThreadData *m = (MotorThreadData *)arg;
  ROS_INFO("%s thread opened!", m->name);

  int setDir = 99999;
  double duty_cycle = 0;

  GPIO::PWM pwm(m->drive_channel, 50);
  pwm.start(0);

  const int SLEEP_PERIOD = 40; // in ms

#ifdef ENCODED_SPEED_MODE
  const unsigned int BIN_SIZE = 15;
  unsigned int prev_enc_sig = 0U;
  const float CYCLE_PERIOD = 0.001f * SLEEP_PERIOD * BIN_SIZE;
  unsigned int sigbin[BIN_SIZE], si = 0, sigtot = 0, diff;
  memset(sigbin, 0, sizeof(sigbin));

  float prev_error = -500, error = 0, int_err = 0, dt_err = 0;
  float prev_power = 0;

  float switch_rate = 0.1f, swr;
  bool dc_asc = true;
#endif

  struct timespec tt, tc;
  clock_gettime(CLOCK_MONOTONIC, &tt);

  while (!m->doExit) {
    pwm.ChangeDutyCycle(duty_cycle);

    {
      // Stop this from continuing on if main thread collapses
      // Not implemented atm -- maybe not at all
    }

    // Sleep
    tt.tv_nsec += SLEEP_PERIOD * 1e6;
    while (tt.tv_nsec >= 1e9) {
      tt.tv_nsec -= (long)1e9;
      ++tt.tv_sec;
    }
    clock_gettime(CLOCK_MONOTONIC, &tc);
    int ud = 1000 * (1e3 * (tt.tv_sec - tc.tv_sec) + 1e-6 * (tt.tv_nsec - tc.tv_nsec));
    // if (!strcmp(m->name, "MotorL"))
    //   printf("%i = %.2f + %.2f\n", ud, 1e3 * (tt.tv_sec - tc.tv_sec), 1e-6 * (tt.tv_nsec - tc.tv_nsec));
    if (ud > 0)
      usleep(ud);

    if (m->doPause) {
      pwm.ChangeDutyCycle(0);
      // GPIO::output(m->drive_channel, 0);
      // ensure_set_value(m->drive_channel, 0);
      continue;
    }
    if (!m->power) {
      duty_cycle = 0;
#ifdef ENCODED_SPEED_MODE
      prev_power = 0.0;
#endif
      continue;
    }

#ifdef ENCODED_SPEED_MODE
    if (!prev_power) {
      duty_cycle = 100.0 * m->power * (double)max_speed / MOTOR_SPEED_MAX;

      // Clear Sampling
      for (si = 0; si < BIN_SIZE; ++si)
        sigbin[si] = 0;

      prev_enc_sig = m->enc_register;
      sigtot = 0;
      int_err = 0;
      prev_error = -m->power * max_speed;
      swr = 0.3f;
    }
    prev_power = m->power;

    // DEBUG
    // int c = !strcmp(m->name, "MotorL");
    // m->target_speed = (strcmp(m->name, "MotorL")) ? 1600.f : 800.f;
    // printf("%s] c=%i ts=%.1f\n", m->name, c, m->target_speed);
    // if (!strcmp(m->name, "MotorR") && si == 9) {
    //   printf("dir:%i\n", m->dir_channel);
    // }
    // DEBUG

    // Sample the wheel rotation speed
    ++si;
    if (si >= BIN_SIZE)
      si -= BIN_SIZE;

    diff = m->enc_register - prev_enc_sig;
    prev_enc_sig = m->enc_register;
    sigtot += diff - sigbin[si];
    sigbin[si] = diff;

    // Estimate speed in mm.s-1
    // -- 225 encoder signals per turn - 31.459cm per turn : 1.4 mm per signal
    m->est_speed = 1.4f * sigtot / CYCLE_PERIOD;
    if (m->dir != setDir) {
      // Wrong way
      setDir = m->dir;
      GPIO::output(m->dir_channel, setDir);
      swr = 0.3f;
      continue;
    }

    // Determine the duty cycle
    float target_speed = m->power * max_speed;
    float error = m->est_speed - target_speed;

    // Proportional
    float pe = -error * 0.0025f;

    // Rate of Change Exponential
    // -- Make the modification by the proportional adjustment component related by a factor
    // -- of the distance between est. & target speeds.
    float exp_mult = MAX(1.f, 1.f + (pow(abs(target_speed - m->est_speed) / 220.f, 2.2f) - 1.f) * 0.07f);
    pe *= exp_mult;

    // Integral
    int_err = int_err * 0.90f + error; //(error < 0 ? -1.f : 1.f) * error * error;
    float ie = -int_err * 0.000235f;

    // Derivative Error
    dt_err = error - prev_error;
    prev_error = error;
    float de = -dt_err * 0.027f;

    // Oscillation Prevention
    // -- Prevent duty-cycle from rapidly going up and down too often
    float dc_delta = (pe + ie + de);
    switch_rate = MIN(1.f, switch_rate * 0.995f + 0.05f * swr);
    swr *= 0.95f;
    if ((dc_delta > 0) != dc_asc) {
      dc_asc = !dc_asc;
      swr += 0.13f;
    }
    float swm = MAX(1.f - pow(switch_rate, 2.8f), pow(abs(target_speed - m->est_speed) / 1600, 2.f));
    // dc_delta *= swm;

#ifdef TEST_NOISE_MODE
    // DEBUG
    if (!strcmp(m->name, "MotorL")) {
      if (m->debug.iterations_left) {
        --m->debug.iterations_left;
        if (!m->debug.iterations_left)
          m->power = 0;

        m->debug.est_speeds[m->debug.iterations_left] = m->est_speed;
      }

      static int iter = 0;
      static float terr = 0.f;
      static float merr = 0.f;

      float esp = m->est_speed;
      // ++iter;
      // terr += esp;
      // merr = MAX(merr, esp);
      // if (iter >= 800) {
      //   printf("[RESULT]: Max.Err:%.0f Avg.Err: %.0f\n", merr, terr / iter);
      //   shutdown_requested = true;
      //   break;
      // }
      // else {
      if (si % 25 == 0) {
        float mLpwr = motorL.power;
        float mRpwr = motorR.power;
        float mLest = motorL.est_speed;
        float mRest = motorR.est_speed;
        // ROS_INFO("Speed: motorL:[%.2f]%.2f(%.2f) motorR:[%.2f]%.2f(%.2f)", mLpwr, mLest, mLpwr * max_speed,
        //          (float)mRpwr, mRest, mRpwr * max_speed);
        // float esp = m->est_speed;
        printf("%s] DC:%.1f tar:%.2f est:%.2f err:%.2f > P:%.2f(%.2f) IE:%.2f DE:%.2f Dir:%i SWM:%.2f\n", m->name,
               duty_cycle, target_speed, esp, error, pe, exp_mult, ie, de, m->dir, swm);
      }
    }
// DEBUG
#endif

    // Update & Clamp
    duty_cycle += (double)dc_delta;
    if (duty_cycle > 100)
      duty_cycle = 100;
    else if (duty_cycle < 0)
      duty_cycle = 0;
#ifdef TEST_NOISE_MODE
    duty_cycle = 0;
#endif
#else
    if (m->dir != setDir) {
      // Wrong way
      setDir = m->dir;
      GPIO::output(m->dir_channel, setDir);
      continue;
    }

    duty_cycle = m->power * max_speed;
#endif
  }

  pwm.stop();
  // GPIO::output(m->drive_channel, 0);
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
    motorL.power = 0;
    motorR.power = 0;
  }

  if (mode == CONTROLLER_MODE_AUTONOMOUS) {
    motorL.power = 1;
    motorL.dir = 1;
    motorR.power = 1;
    motorR.dir = 1;
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

int debug_speeds[] = {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200};
int debug_index = 0;
uint64_t next_debug_time;

// 200ms per retry
bool init_nrf24(int retries)
{
  retries = 100;
  while (true) {
    if (!ros::ok() || shutdown_requested)
      return false;

    bool initialized = nrf24.init();
    if (initialized) {
      ROS_INFO("nrf24.init() succeeded");
      break;
    }
    ROS_INFO("nrf24.init() failed");
    --retries;
    if (retries <= 0) {
      ROS_INFO("nrf24.init() abandoned");
      return false;
    }
    usleep(200000);
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
  return true;
}

bool setup(ros::NodeHandle &nh)
{
  current_mode = CONTROLLER_MODE_AUTONOMOUS;
  max_speed = MOTOR_SET_SPEED_1;

  // Image Saver
  save_image_client = nh.serviceClient<std_srvs::Empty>("/image_view/save");
  captureImageState = 0;

  GPIO::setmode(GPIO::BOARD);

  // GPIO::setup(13, GPIO::Directions::OUT);
  GPIO::setup(GPIO_NRF_IRQ, GPIO::Directions::IN);
  ensure_export(GPIO_NRF_CE);
  ensure_export(GPIO_NRF_SS);

  // Begin RF Communication
  // Serial.begin(9600);
  ROS_INFO("initializing...");
  if (!init_nrf24(100))
    return false;
  last_com_time = millis();

  ROS_INFO("nrf24 initialization succeeded");

  GPIO::setup(GPIO_LIFT, GPIO::Directions::OUT);
  GPIO::setup(GPIO_ROTATER, GPIO::Directions::OUT);
  GPIO::setup(GPIO_APWR, GPIO::Directions::OUT);
  GPIO::setup(GPIO_ADIR, GPIO::Directions::OUT);
  GPIO::setup(GPIO_BDIR, GPIO::Directions::OUT);
  GPIO::setup(GPIO_BPWR, GPIO::Directions::OUT);

  GPIO::setup(GPIO_LENC, GPIO::Directions::IN);
  GPIO::setup(GPIO_RENC, GPIO::Directions::IN);

  // Begin Motor Threads
  int rc;

  memset(&motorL, 0, sizeof(MotorThreadData));
  // motorL.doExit = false;
  // motorL.doPause = false;
  // motorL.finished = false;
  motorL.drive_channel = GPIO_APWR;
  motorL.dir_channel = GPIO_ADIR;
  // motorL.dir = 0;
  // motorL.enc_register = 0;
  // motorL.target_speed = 0.f;
  // motorL.est_speed = 0.f;
  motorL.name = "MotorL";
  if ((rc = pthread_create(&motorL.tid, NULL, motorThread, &motorL))) {
    fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
    return EXIT_FAILURE;
  }

  memset(&motorR, 0, sizeof(MotorThreadData));
  // motorR.doExit = false;
  // motorR.doPause = false;
  // motorR.finished = false;
  motorR.drive_channel = GPIO_BPWR;
  motorR.dir_channel = GPIO_BDIR;
  // motorR.dir = 0;
  // motorR.enc_register = 0;
  // motorR.target_speed = 0.f;
  // motorR.est_speed = 0.f;
  motorR.name = "MotorR";
  if ((rc = pthread_create(&motorR.tid, NULL, motorThread, &motorR))) {
    fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
    return EXIT_FAILURE;
  }

#ifdef ENCODED_SPEED_MODE
  GPIO::add_event_detect(GPIO_LENC, GPIO::Edge::RISING, motor_encoding_signal_callback);
  GPIO::add_event_detect(GPIO_RENC, GPIO::Edge::RISING, motor_encoding_signal_callback);
#endif

  // // DEBUG
  // next_debug_time = millis() + 4000;
  // // DEBUG

  rotator_active = false;
  GPIO::output(GPIO_ROTATER, GPIO::LOW);

  return true;
}

void loop()
{
  uint64_t time = millis();

  // // DEBUG
  // if (time >= next_debug_time) {
  //   const int ITERATION_COUNT = 200;
  //   if (debug_index) {
  //     // Analyse the previous data
  //     // -- Time to reach target
  //     float target_speed = (float)debug_speeds[debug_index - 1];
  //     float target_min = target_speed - 10.f - 0.05f * target_speed;
  //     float target_max = target_speed + 10.f + 0.05f * target_speed;

  //     int iterations_to_target = 0;
  //     float reach_peak_value = 0.f;
  //     int reach_peak_iter = 0;
  //     float cumulative = 0.f;
  //     int iterations_within_target_range = 0;

  //     for (int i = ITERATION_COUNT - 1; i >= 0; --i) {
  //       int invi = ITERATION_COUNT - 1 - i;
  //       float iv = motorL.debug.est_speeds[i];

  //       if (!iterations_to_target) {
  //         if (iv >= target_min && iv < target_max)
  //           iterations_to_target = invi;
  //         reach_peak_value = iv;
  //       }
  //       else if (!reach_peak_iter) {
  //         if (iv >= reach_peak_value) {
  //           reach_peak_value = iv;
  //         }
  //         else {
  //           reach_peak_iter = invi;
  //         }
  //       }

  //       cumulative += iv;

  //       if (iv >= target_min && iv < target_max)
  //         ++iterations_within_target_range;
  //     }

  //     float mean = cumulative / ITERATION_COUNT;
  //     float variance_total = 0.f;
  //     for (int i = 0; i < ITERATION_COUNT; ++i) {
  //       variance_total += (motorL.debug.est_speeds[i] - mean) * (motorL.debug.est_speeds[i] - mean);
  //     }
  //     float std_dev = sqrtf(variance_total / ITERATION_COUNT);

  //     printf("\n### Statistics ###\n"
  //            "--Target Speed: %.1f\n"
  //            "--ITERATION_COUNT to Target: %i\n"
  //            "--ITERATION_COUNT to Peak: %i\n"
  //            "--Peak After Target Reach: %.1f\n"
  //            "--Average Speed: %.1f\n"
  //            "--Std. Deviation: %.1f\n"
  //            "--Percent Time in Target Range: %.1f\n",
  //            target_speed, iterations_to_target, reach_peak_iter, reach_peak_value, mean, std_dev,
  //            100.f * iterations_within_target_range / ITERATION_COUNT);
  //   }

  //   // Begin Next
  //   if (debug_index >= sizeof(debug_speeds) / sizeof(debug_speeds[0])) {
  //     shutdown_requested = true;
  //     return;
  //   }

  //   printf("#######################\n\nTesting speed: %i\n", debug_speeds[debug_index]);
  //   motorL.debug.iterations_left = ITERATION_COUNT;
  //   motorL.power = 1.0f;
  //   max_speed = debug_speeds[debug_index++];
  //   next_debug_time = time + 12000;
  // }
  // // DEBUG

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
        last_com_time = time;

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
            motorL.power = 0;
            motorR.power = 0;
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
              motorL.power = 0;
              motorR.power = 0;
            }
            else {
              // Because the motors are orientated in opposite directions
              //  (one pointed towards left, other right)
              motorL.dir = jx > MID ? 1 : 0;
              motorR.dir = jy > MID ? 1 : 0;

              motorL.power = abs((jx - MID) * 1.05f) / MID;
              if (motorL.power > 1.f)
                motorL.power = 1.f;
              motorR.power = abs((jy - MID) * 1.05f) / MID;
              if (motorR.power > 1.f)
                motorR.power = 1.f;
            }

            // TODO -- Not sending a packet uid to verify -- make it cleaner
            // replyComConfirm = false;
            confirmPacketUID = PID_COUNTER_EMAX;
            printf("CONNECT_RCMOVE: A[%i, %.3f] B[%i, %.3f]\n", motorL.dir, motorL.power * max_speed, motorR.dir,
                   motorR.power * max_speed);
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
          // ROS_INFO("Sent Confirm %u", confirmPacketUID);

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

  if (time - last_com_time > 5000) {
    if (time - last_com_time > 20000) {
      if (init_nrf24(10)) {
        last_com_time = time;
      }
      ROS_INFO("com absence: reinitialized nrf24");
    }
    else if (current_mode == CONTROLLER_MODE_RCOVERRIDE) {
      motorL.power = motorR.power = 0.f;
    }
  }

#ifdef TEST_NOISE_MODE
  motorL.power = 1.f;
  // motorR.power = 1.f;
  if (!rotator_active) {
#else
  if (rotator_active != (motorL.power || motorR.power)) {
#endif
    rotator_active = !rotator_active;

    GPIO::output(GPIO_ROTATER, rotator_active ? GPIO::HIGH : GPIO::LOW);
  }
}

void cleanup()
{
  // Shutoff rotator
  if (rotator_active)
    GPIO::output(GPIO_ROTATER, GPIO::LOW);

  // End Motor Threads
  motorL.doExit = motorR.doExit = true;

  // -- Do other stuff while waiting for threads to exit on their own
  // Move any captures into a session folder
  transferSessionCaptures();

  // Ensure Motor Threads ended
  int r = 0;
  while ((!motorL.finished || !motorR.finished) && r < 4000) {
    usleep(500);
    ++r;
  }

  // Check thread-hanging timeout conditions
  if (motorL.finished) {
    ROS_INFO("Trouble shutting down MotorA-Thread");
    // TODO pthread_abort handling
  }
  if (motorR.finished) {
    ROS_INFO("Trouble shutting down MotorB-Thread");
    // TODO pthread_abort handling
  }

  // Unexport gpios
  GPIO::cleanup(GPIO_LENC);
  GPIO::cleanup(GPIO_RENC);

  GPIO::cleanup(GPIO_LIFT);
  GPIO::cleanup(GPIO_ROTATER);
  GPIO::cleanup(GPIO_APWR);
  GPIO::cleanup(GPIO_ADIR);
  GPIO::cleanup(GPIO_BDIR);
  GPIO::cleanup(GPIO_BPWR);

  // GPIO::cleanup(19);
  // GPIO::cleanup(13);
  GPIO::cleanup(GPIO_NRF_IRQ);
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
  while (ros::ok()) {
    loop();
    if (shutdown_requested) {
      ros::shutdown();
      break;
    }

    loop_rate.sleep();
    ros::spinOnce();
    // ROS_INFO("captureImageState=%i", captureImageState);
  }

  // ROS_INFO("cleanup");
  cleanup();

  if (shutdown_requested) {
    // DEBUG TODO
    system("shutdown -P now");
  }

  ROS_INFO("rf_control Exited");
  usleep(1000);
  return 0;
}