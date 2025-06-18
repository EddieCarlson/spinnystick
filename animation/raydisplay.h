#pragma once

#include <FastLED.h>
#include "../common.h"
#include "../angle.h"
#include "../strip.h"
#include <SPI.h>
#include "../imu_init.h"

CRGB imageRays[NUM_RAYS][COL_HEIGHT];

const int upRay = 0;
unsigned long lastUpRayTimestamp = 0;
unsigned long lastDisplayTimestamp = 0;
int lastRayDisplayed = 0;
double lastDisplayAngle = 0;

double rad_per_ray = 2.0 * PI / ((double) NUM_RAYS);

int threshold = 255 + 255 + 255;

void calculateRay(double curDegrees) {
  double cur_ray = fmod(curDegrees * NUM_RAYS / 360, NUM_RAYS);
  int cur_ray_idx = round(cur_ray);
  CRGB *nearest_ray = imageRays[cur_ray_idx];
  lastRayDisplayed = cur_ray_idx;
  lastDisplayTimestamp = micros();
  // int under_ray_idx = floor(cur_ray);
  // int over_ray_idx = (under_ray_idx + 1) % NUM_RAYS;
  
  // CRGB *under_ray = imageRays[under_ray_idx];
  // CRGB *over_ray = imageRays[over_ray_idx];

  // double dist_to_under = cur_ray - under_ray_idx;
  // double to_under_score = pow(dist_to_under, 2);
  // double to_over_score = pow((1.0 - dist_to_under), 2);

  // double over_weight = to_under_score / to_over_score;
  // double under_weight = 1 - over_weight;

  for(int px = 0; px < COL_HEIGHT; px++) {
    // double blend_r = sqrt(pow(under_ray[px].r * under_weight, 2) + pow(over_ray[px].r * over_weight, 2));
    // double blend_g = sqrt(pow(under_ray[px].g * under_weight, 2) + pow(over_ray[px].g * over_weight, 2));
    // double blend_b = sqrt(pow(under_ray[px].b * under_weight, 2) + pow(over_ray[px].b * over_weight, 2));

    // uint8_t r = min(255, max(0, round(brightness_factor * blend_r)));
    // uint8_t g = min(255, max(0, round(brightness_factor * blend_g)));
    // uint8_t b = min(255, max(0, round(brightness_factor * blend_b)));

    // pixels[px] = CRGB(r, g, b);
    CRGB cpx = nearest_ray[px];
    
    pixels[px] = CRGB(floor(cpx.r * brightness_factor), floor(cpx.g * brightness_factor), floor(cpx.b * brightness_factor));
  }
}

void setAll(CRGB color) {
  for (int rays = 0; rays < NUM_RAYS; rays++) {
    for (int px = 0; px < COL_HEIGHT; px++) {
      imageRays[rays][px] = color;
    }
  }
}

void setTop(CRGB color) {
  for (int rays = NUM_RAYS - 90; rays != 91; rays = (rays + 1) % NUM_RAYS ) {
    for (int px = 0; px < COL_HEIGHT; px++) {
      imageRays[rays][px] = color;
    }
  }
}

double gyroAdjustedAngle() {
  // unsigned long upTimestampDiff = (lastUpRayTimestamp - upTimestamp) / 1000000.0;
  // double upDegreeDiff = curDegPerSec * upTimestampDiff;
  // double curDisplayDegrees = lastDisplayAngle + (curDegPerSec * (micros() - lastDisplayTimestamp) / 1000000.0);
  // double curDegreeDiff = curDisplayDegrees - curAngleEstimate;
  // double weightedDegreeDiff = (upDegreeDiff * 0.65) + (curDegreeDiff * 0.35);
  // if (weightedDegreeDiff > 15) {
  //   double period = 360.0 / curDegPerSec; // 300 millis
  //   double diffForNextDisplay = (weightedDegreeDiff / period) / 3.0;
  //   return fmod(curDisplayDegrees + diffForNextDisplay, 360.0);
  // } else {
  //   return fmod(curDisplayDegrees, 360.0);
  // }
  return fmod(180 + getCurAngle(), 360);
}

void calculate_current_ray() {
  // if (lastUpRayTimestamp == 0) {
  //   lastUpRayTimestamp = micros();
  // }
  // calculateRay(gyroAdjustedAngle());
  // calculateRay(curAngle());
}

void displayCurImageRay() { // takes about 290 micros (@ 32MHz)
  // calculateRay(curAngle()); // takes about 30 micros (12% of time - 242)
  if (currentlySpinning) {
    calculateRay(gyroAdjustedAngle());
  } else {
    clearPixels();
  }
  displayPixels();
}
