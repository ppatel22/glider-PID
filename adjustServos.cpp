#include "adjustServos.h"

int maxAngle = 75;
int minAngle = -75;
// Calculate maximum error using constant1 * 180 + constant2 * 180 + inf for integral?

void adjustPitchServo(double PIDoutput) {
  PIDoutput = constrain(PIDoutput, minAngle, maxAngle);
  pitchServo.write(PIDoutput);
}

void adjustYawServo(double PIDoutput) {
  // Adjust yaw based on error
  yawServo.write(PIDoutput);
}

void adjustRollServo(double PIDoutput) {
  // Adjust roll based on error
  rollServo.write(PIDoutput);
}