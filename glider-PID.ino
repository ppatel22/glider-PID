#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include "adjustServos.h"
#include "sensors.h"
#include "PID.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPL3115A2.h>

extern SoftwareSerial GPSSerial(2, 3);
extern Adafruit_GPS GPS(&GPSSerial);
extern Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
extern Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

Servo pitchServo;
Servo yawServo;
Servo rollServo;

// PID constants for pitch control
double Kp_pitch = 1.0;
double Ki_pitch = 0.1;
double Kd_pitch = 0.01;
double previousPitchError = 0.0;
double integralPitch = 0.0;
double derivativePitch = 0.0;

// PID constants for yaw control
double Kp_yaw = 1.0;
double Ki_yaw = 0.1;
double Kd_yaw = 0.01;
double previousYawError = 0.0;
double integralYaw = 0.0;
double derivativeYaw = 0.0;

// PID constants for roll control
double Kp_roll = 1.0;
double Ki_roll = 0.1;
double Kd_roll = 0.01;
double previousRollError = 0.0;
double integralRoll = 0.0;
double derivativeRoll = 0.0;

// Target
const double targetLatitude = 0.0;
const double targetLongitude = 0.0;
const double targetAltitude = 0.0;
double pitchDesired = 0.0;
double yawDesired = 0.0;
const double rollDesired = 0.0;

// Current position
double currLatitude = 0.0;
double currLongitude = 0.0;
double currAltitude = 0.0;
double pitchCurrent = 0.0;
double yawCurrent = 0.0;
double rollCurrent = 0.0;

// Other constants
const int pitchServoPin = 1;
const int yawServoPin = 2;
const int rollServoPin = 3;
const int controlLoopInterval = 1000;
extern double prevLatitude = 0.0;
double prevLongitude = 0.0;
double prevAltitude = 0.0;

// if this shit don't work, I mixed up latitude and longitude
double calculateAngle(double x, double y, double z) {
  // Avoid division by zero
  if (y == 0.0) {
    return 90.0;  // or any other suitable value for vertical vectors
  }

  double angleRad = atan(sqrt(x * x + z * z) / y);
  double angleDeg = angleRad * (180.0 / M_PI);

  return angleDeg;
}

void calculateAngles() {
  double errorX = targetLatitude - currLatitude;h
  double errorY = targetAltitude - currAltitude;
  double errorZ = targetLongitude - currLongitude;

  double deltaX = currLatitude - prevLatitude;
  double deltaY = currLongitude - prevLongitude;
  double deltaZ = currAltitude - prevAltitude;

  pitchDesired = calculateAngle(errorX, errorY, errorZ);
  pitchCurrent = calculateAngle(deltaX, deltaY, deltaZ);

  yawCurrent = calculateAngle(deltaX, deltaZ, deltaY);  // Swapped y and z to get angle wrt positive z axis
  yawDesired = calculateAngle(errorX, errorZ, errorY);  // Swapped y and z to get angle wrt positive z axis
}

void runPIDControlLoop() {
  // Read sensor data and update PID setpoints
  readSensorData(currLatitude, currLongitude, currAltitude, rollCurrent);
  calculateAngles();

  // Calculate PID and adjust servos
  double pitchOutput = PID(pitchDesired, pitchCurrent, integralPitch, derivativePitch, previousPitchError, Kp_pitch, Ki_pitch, Kd_pitch);
  adjustPitchServo(pitchOutput);

  double yawOutput = PID(yawDesired, yawCurrent, integralYaw, derivativeYaw, previousYawError, Kp_yaw, Ki_yaw, Kd_yaw);
  adjustYawServo(yawOutput);

  double rollOutput = PID(rollDesired, rollCurrent, integralRoll, derivativeRoll, previousRollError, Kp_roll, Ki_roll, Kd_roll);
  adjustRollServo(rollOutput);

  // Save latitude and longitude
  prevLatitude = currLatitude;
  prevLongitude = currLongitude;
  prevAltitude = currAltitude;

  // Wait for a fixed interval
  delay(controlLoopInterval);
}

void setup() {
  // Initialize pins
  pinMode(pitchServoPin, OUTPUT);
  pinMode(yawServoPin, OUTPUT);
  pinMode(rollServoPin, OUTPUT);

  // Initialize servos
  pitchServo.attach(pitchServoPin);
  yawServo.attach(yawServoPin);
  rollServo.attach(rollServoPin);

  pitchServo.write(75);
  yawServo.write(75);
  rollServo.write(75)


  Serial.begin(115200);
  while (!Serial) delay(10);
  // Initialize GPS
  GPSSerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);  // swap with line above to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055! Check your wiring and try again.");
    while (1)
      ;
  }

  // Initialize Barometer
  if (!baro.begin()) {
    Serial.println("Could not find barometer. Check wiring.");
    while (1)
      ;
  }

  delay(1000);
}

void loop() {
  runPIDControlLoop();
}