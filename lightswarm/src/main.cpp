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


}

void loop() {
  // put your main code here, to run repeatedly:
}