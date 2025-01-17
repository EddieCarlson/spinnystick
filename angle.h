#pragma once

#include <Arduino.h>
#include "common.h"
#include "imu_init.h"

double micros_per_rotation = 320000.0;
unsigned long lastMicros = micros();
unsigned long lastGyroMicros = micros();
double lastAngle = 0;
int updates = 0;
double checkGyroIntervalMicros = 25000.0;
bool useGyro = true;

double curAngle() { 
  unsigned long curMicros = micros();
  unsigned long diff = curMicros - lastMicros;
  // if ((diff > checkGyroIntervalMicros) && useGyro) {
  //   updateGyro();
  //   unsigned long afterGyroUpdate = micros();
  // }
  double rotation_fraction = ((double) diff) / micros_per_rotation;
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
