#pragma once

#include <FastLED.h>
#include "../common.h"
#include "../angle.h"
#include "../strip.h"
#include <SPI.h>

CRGB rays[NUM_RAYS][COL_HEIGHT];

CRGB rayFramesToDisplay[COL_HEIGHT];

double rad_per_ray = 2.0 * PI / ((double) NUM_RAYS);

uint32_t rgb_to_hex(uint32_t r, uint32_t g, uint32_t b) {
  return ((r << 16) & 0xFF0000) | ((g << 8) & 0xFF00) | (b & 0xFF);
}

int threshold = 255 + 255 + 255;
double brightness_factor = 0.07; // 0 - 1

uint32_t brightness32 = 0xFB000000;

inline uint32_t getFrame(uint8_t r, uint8_t g, uint8_t b) {
  return brightness32 | ((uint32_t) (b << 16) & 0xFF0000) | ((uint32_t) ((g << 8) & 0xFF00)) | ((uint32_t) ((r & 0xFF)));
}

inline uint32_t getFrame(CRGB color) {
  return getFrame(color.r, color.g, color.b);
}

void calculate_ray(double cur_rad) {
  double cur_ray = fmod(((double) NUM_RAYS) * cur_rad / (2 * PI), NUM_RAYS);
  int under_ray_idx = floor(cur_ray);
  int over_ray_idx = (under_ray_idx + 1) % NUM_RAYS;
  
  CRGB *under_ray = rays[under_ray_idx];
  CRGB *over_ray = rays[over_ray_idx];

  double dist_to_under = cur_ray - under_ray_idx;
  double to_under_score = pow(dist_to_under, 2);
  double to_over_score = pow((1.0 - dist_to_under), 2);

  double over_weight = to_under_score / to_over_score;
  double under_weight = 1 - over_weight;

  for(int px = 0; px < COL_HEIGHT; px++) {
    double blend_r = sqrt(pow(under_ray[px].r * under_weight, 2) + pow(over_ray[px].r * over_weight, 2));
    double blend_g = sqrt(pow(under_ray[px].g * under_weight, 2) + pow(over_ray[px].g * over_weight, 2));
    double blend_b = sqrt(pow(under_ray[px].b * under_weight, 2) + pow(over_ray[px].b * over_weight, 2));

    uint8_t r = min(255, max(0, round(brightness_factor * blend_r)));
    uint8_t g = min(255, max(0, round(brightness_factor * blend_g)));
    uint8_t b = min(255, max(0, round(brightness_factor * blend_b)));

    rayFramesToDisplay[px] = CRGB(r, g, b);
  }
}

void calculate_current_ray() {
  calculate_ray(curAngle());
}

void displayRaySPI() {
  SPI.transfer32((uint32_t) 0);
  for(int i = 0; i < COL_HEIGHT; i++) {
    SPI.transfer32(getFrame(rayFramesToDisplay[i]));
  }
  for(int i = 0; i < 8; i++) {
    SPI.transfer32(brightness32);
  }
  for(int i = COL_HEIGHT - 1; i >= 0; i--) {
    SPI.transfer32(getFrame(rayFramesToDisplay[i]));
  }
  for (uint16_t i = 0; i < (COL_HEIGHT + 14)/16; i++) {
    SPI.transfer(0);
  }
  return;
}

void display_ray_image() { // takes about 290 micros (@ 32MHz)
  calculate_ray(curAngle()); // takes about 30 micros (12% of time - 242)
  displayRaySPI();
}
