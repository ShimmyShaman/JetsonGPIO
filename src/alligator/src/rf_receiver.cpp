#include <ros/ros.h>

#include <termios.h>
// #include "ros_compat.h"

// #include <errno.h>
// #include <fcntl.h>
// #include <poll.h>
// #include <pthread.h>
// #include <stdio.h>
// // #include <stdint.h> ?? TODO
// #include <stdlib.h>
// #include <string.h>
// #include <time.h>
#include <unistd.h>

#include "JetsonNanoRadiohead/RH_NRF24.h"
#include "JetsonNanoRadiohead/RHutil/JetsonNano_gpio.h"
// #include "RH_NRF24.h"

// #define GPIO_LIFT 149
// #define GPIO_ROTATER 200
// #define GPIO_APWR 168
// #define GPIO_ADIR 51
// #define GPIO_BPWR 12
// #define GPIO_BDIR 76

// SECTION
// This section must be kept in sync between the Nano-side Collector App and the
// jsapp
#define SIG_LEN 3
uint8_t signature[] = {113, 176, 87};

enum CommunicationType {
  COMMUNICATION_NULL = 0,
  COMMUNICATION_CONNECT_AUTO,
  COMMUNICATION_CONNECT_RCOVER,
  COMMUNICATION_NANO_SHUTDOWN,
  COMMUNICATION_SPEED_SET_1,
  COMMUNICATION_SPEED_SET_2,
  COMMUNICATION_SPEED_SET_3,
  COMMUNICATION_SPEED_SET_4,
  COMMUNICATION_SPEED_SET_5,
  COMMUNICATION_RCMOVE,
};

enum ControllerModeType {
  CONTROLLER_MODE_NULL = 0,
  CONTROLLER_MODE_AUTONOMOUS = 2,
  CONTROLLER_MODE_RCOVERRIDE = 5,
};
// END-SECTION

// #define MOTOR_SET_SPEED_1 100 //1
// #define MOTOR_SET_SPEED_2 200 //6
// #define MOTOR_SET_SPEED_3 400 //36
// #define MOTOR_SET_SPEED_4 800 //
// #define MOTOR_SET_SPEED_5 1000

// typedef struct MotorThreadData
// {
//   pthread_t tid;
//   unsigned int drive_gpio, dir_gpio;
//   int *maxSpeed;
//   float power; // (0->1)
//   int dir;
//   bool doExit;
//   bool doPause;
//   bool finished;
// } MotorThreadData;

// MotorThreadData motorA, motorB;

// // Singleton instance of the radio driver
// // RH_NRF24 nrf24;
// // RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// // RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// // RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini
RH_NRF24 nrf24(13, 19); // For the Nvidia Jetson Nano (gpio pins 13, 19 are J41
                        // 22, 24 respectively)

// int maxSpeed;
// enum ControllerModeType cmode;

// void *motorThread(void *arg)
// {
//   MotorThreadData *m = (MotorThreadData *)arg;

//   int setDir = 99999;

//   unsigned long spupdate = millis();

//   while (!m->doExit)
//   {
//     if (m->doPause)
//     {
//       ensure_set_value(m->drive_gpio, 0);
//       usleep(1000);
//     }
//     else
//     {
//       // TODO -- stop this from continuing on if main thread collapses
//       // struct timespec current;
//       // clock_gettime(CLOCK_REALTIME, &current);

//       // if(m->)
//     }

//     if (*m->maxSpeed && m->power)
//     {
//       if (m->dir != setDir)
//       {
//         setDir = m->dir;
//         ensure_set_value(m->dir_gpio, setDir);
//       }

//       int mex = (int)(m->power * *m->maxSpeed);

//       if (millis() - spupdate > 1000)
//       {
//         spupdate = millis();
//         //printf("mex=%i\n", mex);
//       }

//       ensure_set_value(m->drive_gpio, 1);
//       usleep(mex);
//       if (mex < 1000)
//       {
//         ensure_set_value(m->drive_gpio, 0);
//         usleep(1000 - mex);
//       }
//       else
//       {
//         continue;
//       }
//     }
//     else
//       ensure_set_value(m->drive_gpio, 0);
//   }

//   ensure_set_value(m->drive_gpio, 0);
//   m->finished = true;
//   puts("thread-closed");
//   pthread_exit(NULL);
// }

// void changeMode(enum ControllerModeType mode)
// {
//   if (mode == CONTROLLER_MODE_RCOVERRIDE && cmode == CONTROLLER_MODE_AUTONOMOUS)
//   {
//     maxSpeed = 0;
//   }

//   cmode = mode;

//   if (mode == CONTROLLER_MODE_AUTONOMOUS)
//   {
//     maxSpeed = 0;
//   }
// }

bool setup()
{
  //   cmode = CONTROLLER_MODE_AUTONOMOUS;
  //   maxSpeed = MOTOR_SET_SPEED_1;

  ensure_export(13);
  ensure_export(19);

  //   // Begin RF Communication
  //   // Serial.begin(9600);
  //   puts("initializing...");
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

  //   // ensure_export(GPIO_LIFT);
  //   // ensure_set_dir(GPIO_LIFT, OUTPUT);
  //   // ensure_export(GPIO_ROTATER);
  //   // ensure_set_dir(GPIO_ROTATER, OUTPUT);
  //   ensure_export(GPIO_APWR);
  //   ensure_set_dir(GPIO_APWR, OUTPUT);
  //   ensure_export(GPIO_ADIR);
  //   ensure_set_dir(GPIO_ADIR, OUTPUT);
  //   // ensure_export(GPIO_BDIR);
  //   // ensure_set_dir(GPIO_BDIR, OUTPUT);
  //   // ensure_export(GPIO_BPWR);
  //   // ensure_set_dir(GPIO_BPWR, OUTPUT);

  //   // Begin Motor Threads
  //   int rc;

  //   motorA.doExit = false;
  //   motorA.doPause = false;
  //   motorA.finished = false;
  //   motorA.drive_gpio = GPIO_APWR;
  //   motorA.dir_gpio = GPIO_ADIR;
  //   motorA.maxSpeed = &maxSpeed;
  //   motorA.dir = 0;
  //   motorA.power = 0.f;
  //   if ((rc = pthread_create(&motorA.tid, NULL, motorThread, &motorA)))
  //   {
  //     fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
  //     return EXIT_FAILURE;
  //   }
  //   puts("thread-opened");

  //   // motorB.doExit = false;
  //   // motorB.doPause = false;
  //   motorB.finished = true;
  //   // motorB.finished = false;
  //   // motorB.drive_gpio = GPIO_BPWR;
  //   // motorB.dir_gpio = GPIO_BDIR;
  //   // motorB.maxSpeed = &maxSpeed;
  //   // motorB.dir = 0;
  //   // motorB.power = 0.f;
  //   // if ((rc = pthread_create(&motorB.tid, NULL, motorThread, &motorB)))
  //   // {
  //   //   fprintf(stderr, "error: pthread_create, rc: %d\n", rc);
  //   //   return EXIT_FAILURE;
  //   // }
  //   // puts("thread-opened");

  return true;
}

void loop()
{
  //   // puts("Sending to nrf24_server");
  //   // // Send a message to nrf24_server
  //   // uint8_t data[] = "Hello World!";
  //   // nrf24.send(data, sizeof(data));

  //   // nrf24.waitPacketSent();
  //   // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // ROS_INFO("waitAvailableTimeout");
  if (nrf24.available()) {
    // Should be a reply message for us now
    if (nrf24.recv(buf, &len)) {
      // printf("got reply: ");
      // puts((char *)buf);

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
        ROS_INFO("Verified Communication Received");
        enum CommunicationType ct = (enum CommunicationType)buf[SIG_LEN];

        bool replyComConfirm = true;
        switch (ct) {
          case COMMUNICATION_CONNECT_AUTO:
            // changeMode(CONTROLLER_MODE_AUTONOMOUS);
            puts("CONNECT_AUTO");
            break;
          case COMMUNICATION_CONNECT_RCOVER:
            // changeMode(CONTROLLER_MODE_RCOVERRIDE);
            puts("CONNECT_RCOVER");
            break;
          case COMMUNICATION_RCMOVE: {
            // changeMode(CONTROLLER_MODE_RCOVERRIDE);

            // // TODO check this is okay with 127 no movement
            // motorA.power = ((float)buf[SIG_LEN + 1] - 127.f) / 128.f;
            // if (motorA.power < 0)
            // {
            //   motorA.dir = 1;
            //   motorA.power = -motorA.power;
            // }
            // else
            // {
            //   motorA.dir = 0;
            // }
            // motorB.power = ((float)buf[SIG_LEN + 2] - 127.f) / 128.f;
            // if (motorB.power < 0)
            // {
            //   motorB.dir = 1;
            //   motorB.power = -motorB.power;
            // }
            // else
            // {
            //   motorB.dir = 0;
            // }

            // TODO -- Not sending a packet uid to verify -- make it cleaner
            // replyComConfirm = false;
            printf("CONNECT_RCMOVE\n");// : A[%i, %.3f] B[%i, %.3f]\n", motorA.dir, motorA.power, motorB.dir,
            // motorB.power);
          } break;
          case COMMUNICATION_NANO_SHUTDOWN:
            puts("TODO -- Nano Shutdown");
            break;
          case COMMUNICATION_SPEED_SET_1:
            // maxSpeed = MOTOR_SET_SPEED_1;
            puts("Speed Set to MOTOR_SET_SPEED_1");
            break;
          case COMMUNICATION_SPEED_SET_2:
            // maxSpeed = MOTOR_SET_SPEED_2;
            puts("Speed Set to MOTOR_SET_SPEED_2");
            break;
          case COMMUNICATION_SPEED_SET_3:
            // maxSpeed = MOTOR_SET_SPEED_3;
            puts("Speed Set to MOTOR_SET_SPEED_3");
            break;
          case COMMUNICATION_SPEED_SET_4:
            // maxSpeed = MOTOR_SET_SPEED_4;
            puts("Speed Set to MOTOR_SET_SPEED_4");
            break;
          case COMMUNICATION_SPEED_SET_5:
            // maxSpeed = MOTOR_SET_SPEED_5;
            puts("Speed Set to MOTOR_SET_SPEED_5");
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
          data[SIG_LEN] = buf[SIG_LEN + 1];

          nrf24.send(data, sizeof(data));
          nrf24.waitPacketSent();

          // puts("Sent Connectivity Response");
        }
      }
    }
    else {
      puts("recv failed");
    }
  }
  else {
    // puts("No reply, is nrf24_server running?");
  }
}

void cleanup()
{
  //   // End Motor Threads
  //   motorA.doExit = motorB.doExit = true;

  //   int r = 0;
  //   while ((!motorA.finished || !motorB.finished) && r < 4000)
  //   {
  //     usleep(500);
  //     ++r;
  //   }

  //   if (motorA.finished)
  //   {
  //     puts("Trouble shutting down MotorA-Thread");
  //   }
  //   if (motorB.finished)
  //   {
  //     puts("Trouble shutting down MotorB-Thread");
  //   }
  //   // TODO -- unexport??

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
  ROS_INFO("rf_receiver Begun");

  ROS_INFO("Press 'q' to exit");
  ros::init(argc, argv, "rf_receiver");
  ros::NodeHandle nh;

  if (!setup()) {
    cleanup();

    ROS_INFO("rf_receiver Failed Setup");
    return -1;
  }

  ros::Rate loop_rate(2);
  while (ros::ok()) {
    // Call your non-blocking input function
    // int c = getch();
    // if (c == 'q') {
    //   break;
    // }

    // ros::spinOnce();
    loop_rate.sleep();
    // ROS_INFO("a");
    loop();
  }

  // puts("cleanup");
  cleanup();

  // ROS_INFO("rf_receiver Exited");
  return 0;
}

// int main(int argc, char **argv)
// {
//   return 0;
//   puts("Entered App...");
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

//   puts("exiting...");
//   exit(0);
// }