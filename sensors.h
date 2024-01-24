#ifndef SENSORS_H
#define SENSORS_H
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MPL3115A2.h>

#define GPSECHO false

extern SoftwareSerial GPSSerial;
extern Adafruit_GPS GPS;
extern Adafruit_BNO055 bno;
extern Adafruit_MPL3115A2 baro;

void readSensorData(double currLatitude, double currLongitude, double currAltitude, double rollCurrent);

#endif