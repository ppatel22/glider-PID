#ifndef PID_H
#define PID_H
#include <Arduino.h>

extern double prevLatitude;

double PID(double desired, double current, double integral, double derivative, double previous, double Kp, double Ki, double Kd);

#endif