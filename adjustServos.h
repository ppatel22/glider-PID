#ifndef ADJUSTSERVOS_H
#define ADJUSTSERVOS_H
#include <Servo.h>
#include <Arduino.h>

extern Servo pitchServo;
extern Servo yawServo;
extern Servo rollServo;

void adjustPitchServo(double PIDoutput);
void adjustYawServo(double PIDoutput);
void adjustRollServo(double PIDoutput);

#endif