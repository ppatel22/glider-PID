#include "sensors.h"

void readSensorData(double currLatitude, double currLongitude, double currAltitude, double rollCurrent) {
  // Read GPS data
  if (GPS.fix) {
    // Serial.print("Location: ");
    // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    // Serial.print(", ");
    // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    // Serial.print("Angle: "); Serial.println(GPS.angle);
    // Serial.print("Altitude: "); Serial.println(GPS.altitude);
    // Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    currLatitude = GPS.latitude;
    currLongitude = GPS.longitude;
    currAltitude = GPS.altitude;  // Ideally swap this out for barometer
  }

  // Read IMU data
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  rollCurrent = orientationData.orientation.x;  // could be x, y, or z

  // Read barometer data
  currAltitude = baro.getAltitude();
}

// Roll is zeroed at the start of the program. All angles should be calculated from this zero