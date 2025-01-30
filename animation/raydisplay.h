#pragma once

#include <FastLED.h>
#include "../common.h"
#include "../angle.h"
#include "../strip.h"
#include <SPI.h>

CRGB imageRays[NUM_RAYS][COL_HEIGHT];

double rad_per_ray = 2.0 * PI / ((double) NUM_RAYS);

int threshold = 255 + 255 + 255;

void calculateRay(double cur_rad) {
  double cur_ray = fmod(((double) NUM_RAYS) * cur_rad / (2 * PI), NUM_RAYS);
  int cur_ray_idx = round(cur_ray);
  CRGB *nearest_ray = imageRays[cur_ray_idx];
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

void calculate_current_ray() {
  calculateRay(curAngle());
}

void displayCurImageRay() { // takes about 290 micros (@ 32MHz)
  calculateRay(curAngle()); // takes about 30 micros (12% of time - 242)
  displayPixels();
}
