#pragma once

#include <Arduino.h>
#include "common.h"
#include "imu_init.h"

 // refreshes per rotation = micros_per_rotation / micros_per_refresh. 300 millis / 300 micros = 1000
double microsPerRotation = 300000.0;
unsigned long lastMicros = micros();
unsigned long lastGyroMicros = micros();
double lastAngle = 0;
int updates = 0;
double checkGyroIntervalMicros = 25000.0;
bool useGyro = true;

int curPeriodIndex = 2;
double milliPeriods[5] = {185, 250, 300, 400, 600};

double rayToRad(int ray) {
  return (((double) ray) * 2.0 * M_PI) / ((double) NUM_RAYS);
}

double curAngle() { 
  unsigned long curMicros = micros();
  unsigned long diff = curMicros - lastMicros;
  // if ((diff > checkGyroIntervalMicros) && useGyro) {
  //   updateGyro();
  //   unsigned long afterGyroUpdate = micros();
  // }
  double rotation_fraction = ((double) diff) / microsPerRotation;
  double rotation_rads = rotation_fraction * 2 * PI;
  double newAngle = rad_mod(lastAngle + rotation_rads);

  updates += 1;
  if (newAngle < lastAngle) { 
    //Serial.println(updates);
    updates = 0;
  }

  lastAngle = newAngle;
  lastMicros = micros();

  return newAngle;
}

void setNextPeriod() {
  microsPerRotation = milliPeriods[curPeriodIndex] * 1000.0;
  curPeriodIndex = (curPeriodIndex + 1) % 5;
}