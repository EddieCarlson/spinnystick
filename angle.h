#include <Arduino.h>

double micros_per_rotation = 580000.0;
unsigned long lastMicros = micros();
double lastAngle = 0;
int updates = 0;

double curAngle() { 
  unsigned long curMicros = micros();
  unsigned long diff = curMicros - lastMicros;
  double rotation_fraction = ((double) diff) / micros_per_rotation;
  double rotation_rads = rotation_fraction * 2 * PI;
  double newAngle = fmod(lastAngle + rotation_rads, 2 * PI);

  updates += 1;
  if (newAngle < lastAngle) { 
    //Serial.println(updates);
    updates = 0;
  }

  lastAngle = newAngle;
  lastMicros = micros();

  return newAngle;
}
