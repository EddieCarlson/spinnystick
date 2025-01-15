#pragma once

#include <FastLED.h>
#include "../common.h"
#include "../angle.h"
#include "../strip.h"

// TODO: not NUMPIXELS but COL_HEIGHT....here and everywhere...

CRGB rays[NUM_RAYS][COL_HEIGHT];

uint32_t rayToDisplay[COL_HEIGHT];

double rad_per_ray = 2.0 * PI / ((double) NUM_RAYS);

uint32_t rgb_to_hex(uint32_t r, uint32_t g, uint32_t b) {
  return ((r << 16) & 0xFF0000) | ((g << 8) & 0xFF00) | (b & 0xFF);
}

void setTestImage() {
  for(int ray = 0; ray < 200; ray++) {
    for(int px = 0; px < COL_HEIGHT; px++) {
      rays[ray][px] = CRGB::Maroon;
    }
  }

  for(int ray = 270; ray < 470; ray++) {
    for(int px = 0; px < (COL_HEIGHT * 0.6); px++) {
      rays[ray][px] = CRGB::DarkTurquoise;
    }
  }

  for(int ray = 540; ray < 740; ray++) {
    for(int px = 0; px < COL_HEIGHT; px++) {
      rays[ray][px] = CRGB::Gold;
    }
  }

  for(int ray = 810; ray < 1010; ray++) {
    for(int px = 0; px < (COL_HEIGHT * 0.6); px++) {
      rays[ray][px] = CRGB::DarkTurquoise;
    }
  }
}

int threshold = 255 + 255 + 40;

void calculate_ray(double cur_rad, uint32_t *ray) {
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
    uint8_t blend_r = max(0, floor(sqrt(pow(under_ray[px].r * under_weight, 2) + pow(over_ray[px].r * over_weight, 2))));
    uint8_t blend_g = max(0, floor(sqrt(pow(under_ray[px].g * under_weight, 2) + pow(over_ray[px].g * over_weight, 2))));
    uint8_t blend_b = max(0, floor(sqrt(pow(under_ray[px].b * under_weight, 2) + pow(over_ray[px].b * over_weight, 2))));

    if (blend_r + blend_g + blend_b < threshold) {
      rayToDisplay[px] = rgb_to_hex(blend_r, blend_g, blend_b);
    } else {
      rayToDisplay[px] = 0;
    }
  }
}

void calculate_current_ray(uint32_t *ray) {
  calculate_ray(curAngle(), ray);
}

void displayRay() {
  for(int i = 0; i < COL_HEIGHT; i++) {
    setBothSides(i, rayToDisplay[i]);
  }
  strip.show();
}

void display_ray_image() {
  calculate_ray(curAngle(), rayToDisplay);
  displayRay();
}
