#include <Arduino.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "defs.h"

/* UDP instance */
WiFiUDP udp;
IPAddress localIP;

/* buffer to hold incoming and outgoing packets */
uint8_t packetBuffer[BUFFERSIZE];

/* create MPU6050 sensor instance */
Adafruit_MPU6050 sensor;

void _establish_wifi_connection(){
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    debugln("[+] Connecting to WiFi...");
    delay(200);
  }

  // show the IP address of server if connected to WIFI
  debugln("+ Connected with Local IP_ADRR:");
  debug(WiFi.localIP()); debugln();

}

void _init_MPU6050(){
  /* Initialize MPU6050 */
  if(!gyroscope.begin(0x68)){
    debugln("MPU6050 allocation failed!");
    for(;;);
  }

  gyroscope.setAccelerometerRange(MPU6050_RANGE_8_G);
  gyroscope.setGyroRange(MPU6050_RANGE_500_DEG);
  gyroscope.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void setup() {
  Serial.begin(115200);
  debugln();

  debugln("----------");
  debugln("Lightswarm");
  debugln("Version:");
  debugln(VERSION);
  debugln("----------");

  Serial.print(F("Compiled at: "));
  Serial.print(F(__TIME__));
  debugln(" ")
  Serial.print(F(__DATE__));

  randomSeed(analogRead(A0));
  debug("Analog read(A0) = ");
  debug(analogRead(A0));

  /* initialize MPU6050*/
  _init_MPU6050();

  /* every deice starts with ID 0 and changes of from there */
  my_swarmid = 0;
  debug("Lightswarm instance: "); debug(my_swarmid); debugln();

  /* connect to WIFI */
  _establish_wifi_connection();

  /* start UDP*/
  debugln("[+] starting UDP ");
  udp.begin(local_port);
  debug("[+] local port: "); debug(udp.local_port());

  /* initialize sensors */
  for(int i = 0; i < SWARMSIZE; i++){
    swarmAddresses[i] = 0;
    swarmClear[i] = 0;
    swarmTimeStamp[i] = -1;
  }

  /* set up swarm device state variables */
  swarmClear[my_swarmid] = 0;
  swarmTimeStamp[my_swarmid] = 1; 
  clearColor = swarmClear[my_swarmid];
  swarmVersion[my_swarmid] = VERSION;
  swarmState[my_swarmid] = master_state;


  // set swarm IDs based on IP address
  localIP = WiFi.localIP();
  swarmAddresses[0] = localIP[3];
  my_swarmid = 0;

  debug("My swarm ID;"); debug(my_swarmid); debugln();

}

void loop() {
  // put your main code here, to run repeatedly:
}