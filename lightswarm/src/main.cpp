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
int serverAddress;

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
  if(!sensor.begin(0x68)){
    debugln("MPU6050 allocation failed!");
    for(;;);
  }

  sensor.setAccelerometerRange(MPU6050_RANGE_8_G);
  sensor.setGyroRange(MPU6050_RANGE_500_DEG);
  sensor.setFilterBandwidth(MPU6050_BAND_5_HZ);
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
  debugln(" ");
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
  debug("[+] local port: "); debug(local_port);


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
  int secondsCount;
  int lastSecondsCount;

  #define LOGHOWOFTEN

  secondsCount = millis() / 100;

  /* read the mpu6050 temperature to determine which has the highest temperature */
  sensors_event_t a, g, temp;
  sensor.getEvent(&a, &g, &temp);

  /* get the actual temperature */


  debugln(temp.temperature);

  /* wait to see if a reply is availabel */
  delay(300);

  int cb = udp.parsePacket();

  if(!cb){
    debugln("No packet returned");

  }else{
    /* packet received. read data from it into a buffer */
    udp.read(packetBuffer, PACKET_SIZE);

    debug("packetBuffer[1] = ");
    debug(packetBuffer[1]);

    if(packetBuffer[1] == LIGHT_UPDATE_PACKET){
      debug("LIGHT UPDATE PACKET received from lightswarm #");
      debugln(packetBuffer[2]);
      setAndReturnMySwarmIndex(packetBuffer[2]);

      debug("LS packet received #");
      debug(packetBuffer[2]);
      debug(" swarmstate");

      if (packetBuffer[2]){
        debug("Slave");
      }else{
        debug("Master");
      }

    }

    if(packetBuffer[1] == RESET_SWARM_PACKET){
      debugln("RESET SWARM PACKET received");
      master_state = true;
      debugln("I just became master");
      digitalWrite(0, LOW);
    }

    if(packetBuffer[1] == RESET_ME_PACKET){
      debugln("RESET ME packet received");

      /* check whether the device to reset is this device */
      if(packetBuffer[2] == swarmAddresses[my_swarmid]){
        master_state = true;
        debugln(" Reset done. I just became master");
        digitalWrite(0, LOW);
      }else{
        debug("Target device is #");
        debug(packetBuffer[2]);
        debuln(": Reset ignored");
      }

    }

    if(packetBuffer[1] == DEFINE_SERVER_LOGGER_PACKET){
      debugln("DEFINE SERVER LOGGER PACKET received");
      serverAddress = IPAddress(packetBuffer[4], packetBuffer[5], packetBuffer[6], packetBuffer[7]);
      debug("Server address received: ");
      debugln(serverAddress);
    }

    if(packetBuffer[1] == BLINK_BRIGHT_RED){
      debugln("BLINK BRIGHT RED packet received");

      /* check whether the device that should blink is this one */
      if(packetBuffer[2] == swarmAddresses[my_swarmid]){
        digitalWrite(0, HIGH);
        delay(300);
        digitalWrite(0, LOW);
        delay(300);
      } else{
        debug("Target device is #");
        debug(packetBuffer[2]);
        debuln(": Blink ignored");
      }
    }

    debug("Master status");

    if(master_state == true){
      digitalWrite(0, LOW);
      debug("MASTER");
    } else{
      digitalWrite(0, HIGH);
      debug("SLAVE");
    }
    
  }

  debug("Server address: "); debugln(serverAddress);
  debugln("----------");

  for(int  i = 0; i < SWARMSIZE; i++){
    debug("swarmAddress[");
    debug(i);
    debug("] = ");
    debugln(swarmAddresses[i]);
  }

  debugln("----------");

  broadcastARandomPacket();
  sendLogToServer();

}