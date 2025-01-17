#pragma once
#include <math.h>

#define NUMPIXELS 200
#define COL_HEIGHT 96
#define ROPE_PIXELS 25
#define NUM_RAYS 1080

const int image_size = (ROPE_PIXELS + COL_HEIGHT) * 2;

double dist2(double x1, double y1, double x2, double y2) {
  return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

double dist(double x1, double y1, double x2, double y2) {
  return pow(dist2(x1, y1, x2, y2), 0.5);
}

double rad_mod(double radians) {
  double modded = fmod(radians, 2.0 * M_PI);
  if (modded > ((2 * M_PI) - 0.0001)) {
    return 0.0;
  } else {
    return modded;
  }
}