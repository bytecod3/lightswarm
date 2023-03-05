#ifndef DEFS_H
#define DEFS_H

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x, y) Serial.printf(x, y)

#else

#define debug(x)
#define debugln(x)
#define debugf(x, y)

#endif

/* WIFI credentials */
char ssid[] = "Eduh";
char password[] = "password2";

#define SWARMSIZE 5
#define VERSION 1

/* 30 seconds timeout to read */
#define SWARMTOOOLD 30 

int my_swarmid = 0;

/*Packet types*/
#define LIGHT_UPDATE_PACKET 0  // contains current light from a swarm device
#define RESET_SWARM_PACKET 1 // all lightswarm devices are told to reset their software
#define CHANGE_TEST_PACKET 2 //
#define RESET_ME_PACKET 3 // just reset a particluar lightswarm device ID
#define DEFINE_SERVER_LOGGER_PACKET 4 // new IP address of RPI so devices can send packets
#define LOG_TO_SERVER_PACKET 5 // packets send from lightswarm device to RPI
#define MASTER_CHANGE_PACKET 6
#define BLINK_BRIGHT_RED 7 // command to a lightswarm device to blink the bright LED on the TCS34725

/* local port to listen for UDP packets */
unsigned int local_port = 2910;

/* master variables */
bool master_state = true; // true if master, false if not

int swarmAddresses[SWARMSIZE];
int swarmClear[SWARMSIZE];
int swarmVersion[SWARMSIZE];
int swarmState[SWARMSIZE];
long swarmTimeStamp[SWARMSIZE];

// variables for light sensor
int clearColor;
int redColor;
int blueColor;
int greenColor;

const int PACKET_SIZE = 14; // light update packet
const int BUFFERSIZE = 1024;

#endif