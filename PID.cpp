#include "PID.h"

double trimError(double angle) {
  if (abs(angle) > 180) {
    angle = (fmod(angle, 180.0) * -1.0);
  }
  if (prevLatitude == 0) {
    angle = 0;   // this hard-coded condition ensures there is no adjustment on the first timestep
  }
}

double PID(double desired, double current, double integral, double derivative, double previous, double Kp, double Ki, double Kd) {
  double error = trimError(desired - current);

  integral += error;
  derivative = error - previous;
  previous = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

