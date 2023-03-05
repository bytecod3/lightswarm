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

/* send a temperature request to all sensors to get their temperature */
unsigned long sendTemperatureUpdatePacket(IPAddress &address){
  /* set all bytes in the buffer to 0*/
  memset(packetBuffer, 0, PACKET_SIZE);

  /* initialize values needed to form a light packet */
  packetBuffer[0] = 0xF0; // start byte
  packetBuffer[1] = LIGHT_UPDATE_PACKET;  // packet type
  packetBuffer[2] = localIP[3]; // swarm number - last part of the IP address
  packetBuffer[3] = master_state; // 0 - slave, 1 - master
  packetBuffer[4] = VERSION; // software version
  
  packetBuffer[5] = 0x0F; // end byte


  // all packets have been given values
  // you can send a packet requesting coordination
  udp.beginMulticastPacket(address, local_port, WiFi.localIP());
  udp.write(packetBuffer, PACKET_SIZE);
  udp.endPacket();

}

void broadcastARandomPacket(){
  int sendToTempSwarm = 255;
  debug(" Broadcast to swarm = ");
  debug(sendToTempSwarm);
  debug(" ");

  int randomdelay = random(0, MAXDELAY);
  debug("delay = "); debug(randomdelay);
  debug("ms : ");

  delay(randomdelay);

  IPAddress sendSwarmAddress(192, 168, 1, sendToTempSwarm);
  sendTemperatureUpdatePacket(sendSwarmAddress);
}


void checkAndSetIfMaster(){
  /* check if a device became master, remove stale and dead devices from the swarm */
  for(int i = 0; i < SWARMSIZE; i++){
    debug("#");
    debug(i);
    debug("/");
    debug(swarmState[i]);
    debug("/");
    debug(swarmVersion[i]);
    debug(":");

    // age data
    int howLongAgo = millis() - swarmTimeStamp[i];

    if(swarmTimeStamp[i] == 0){
      debug("TO ");
    } else if(swarmTimeStamp[i] == -1){
      debug("NP ");
    } else if(swarmTimeStamp[i] == 1){
      debug(" ME ");
    } else if(howLongAgo > SWARMTOOOLD){
      debug("TO ");
      swarmTimeStamp[i] = 0;
      swarmClear[i] = 0;
    }else{
      debug("PR ");
    }
  }

  debugln();
  bool setMaster = true;

  for(int i = 0; i < SWARMSIZE; i++){
    if(swarmClear[my_swarmid] >= swarmClear[i]){
      // i might be master
    }else{
      // not master
      setMaster = false;
      break;
    }
  }

  if(setMaster == true){
    if(master_state == false){
      debug("I have been set to master");
      digitalWrite(0, LOW);
    }
    master_state = true;
  }else{
    if (master_state == true){
      debugln("I lost master");
      digitalWrite(0, HIGH);
    }

    master_state = false;
    
  }

  swarmState[my_swarmid] = master_state;

}


int setAndReturnMySwarmIndex(int incomingID){
  for(int i=0; i<SWARMSIZE; i++){
    if(swarmAddresses[i] == incomingID){
      return i;
    }else if(swarmAddresses[i] == 0){
        /* not in the system, so put it in */
        swarmAddresses[i] = incomingID;
        debug("incomingID ");
        debug(incomingID);
        debug(" assigned #");
        debug(i);
        return i;
    }
    
  }

  // if we get here we have a new member
  // delete the oldest one and add the new one
  int oldSwarmID;
  long oldTime;
  oldTime = millis();
  
  for(int i=0;i < SWARMSIZE; i++){
    if(oldTime > swarmTimeStamp[i]){
      oldTime = swarmTimeStamp[i];
      oldSwarmID = i;
    }
  }

  // remove the old one and put this one in
  swarmAddresses[oldSwarmID] = incomingID;

}

/* send log packet to server if master and server addresss are defined */
void sendLogToServer(){
  // build the string
  char myBuildString[1000];
  myBuildString[0] = '\0';

  if(master_state == true){
    // master is defined
    // now check if server is defined
    if((serverAddress[0] == 0) && (serverAddress[1] == 0)){
      return; // we are done. not defined
    }else{
      // send the packet as a string with the following format:
      // swarmID, MasterSlave, SoftwareVersion
      // 0, 1, 15, 3883, PR | 1, 0, 14, 399, PR
      
      char swarmString[20];
      swarmString[0] = '\0';

      for(int i = 0; i< SWARMSIZE; i++){
        char stateString[5];
        stateString[0] = '\0';

        if(swarmTimeStamp[i] == 0){
          strcat(stateString, "TO");
        } else if (swarmTimeStamp[i] == -1){
          strcat(stateString, "NP");
        } else if(swarmTimeStamp[i] == 1){
          strcat(stateString, "PR");
        }

        sprintf(swarmString, "%i, %i, %i, %i, %s, %i", i, swarmState[i], swarmVersion[i], swarmClear[i], stateString, swarmAddresses[i]);

        strcat(myBuildString, swarmString);
        if(i < SWARMSIZE -1){
          strcat(myBuildString, "|");
        }
      }
    }

    // set all bytes in the buffer to 0
    memset(packetBuffer, 0, BUFFERSIZE);

    // initialize values needed to form teperature packet
    packetBuffer[0] = 0xF0; // start byte
    packetBuffer[1] = LOG_TO_SERVER_PACKET; // packet type
    packetBuffer[2] = localIP[3]; // send swarm number
    packetBuffer[3] = strlen(myBuildString); // length of the string in bytes
    packetBuffer[4] = VERSION; // software version
    
    int i = 0;
    for(i=0; i< strlen(myBuildString); i++){
      packetBuffer[i + 5] = myBuildString[i];
    }

    packetBuffer[i + 5] = 0x0F; // end byte

    debug("Sending log to server");
    debugln(myBuildString);
    int packetLength;
    packetLength = i + 5 + 1;

    udp.beginPacket(serverAddress, local_port);
    udp.write(packetBuffer, packetLength);
    udp.endPacket();

  }
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
        debugln(": Reset ignored");
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
        debugln(": Blink ignored");
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

} // end of loop



