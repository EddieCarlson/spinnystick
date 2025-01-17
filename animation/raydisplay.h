#pragma once

#include <FastLED.h>
#include "../common.h"
#include "../angle.h"
#include "../strip.h"
#include <SPI.h>

// TODO: not NUMPIXELS but COL_HEIGHT....here and everywhere...

// class APAColor {
//   public:

//   uint8_t brightness;
//   uint8_t r;
//   uint8_t g;
//   uint8_t b;

//   APAColor(uint8_t _brightness, uint8_t _r, uint8_t _g, uint8_t _b) {
//     _brightness = 0b11100000 | brightness;
//     r = _r;
//     g = _g;
//     b = _b;
//   }
// };

// class APAStripPayload {
//   uint32_t start;
//   APAColor *colors;
//   uint32_t end;

//   APAStripPayload(APAColors *_colors, )
// }

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

uint32_t brightness_byte_or = ((0b11100000 | BRIGHTNESS) << 24) & 0xFF000000;

void displayRaySPI() {
  SPI.transfer32((uint32_t) 0);
  for(int i = 0; i < COL_HEIGHT; i++) {
    SPI.transfer32((brightness_byte_or | rayToDisplay[i]));
  }
  for(int i = 0; i < 8; i++) {
    SPI.transfer32(brightness_byte_or);
  }
  for(int i = COL_HEIGHT - 1; i >= 0; i--) {
    SPI.transfer32((brightness_byte_or | rayToDisplay[i]));
  }
  // TODO: send FFF multiple times?
  for (uint16_t i = 0; i < (COL_HEIGHT + 14)/16; i++) {
    SPI.transfer(0);
  }
  // digitalWrite(DATAPIN, LOW);
  // pinMode(DATAPIN, OUTPUT);
  // digitalWrite(CLOCKPIN, LOW);
  // pinMode(CLOCKPIN, OUTPUT);
  // SPI.transfer32(0);
  return;
}

void displayRay() {
  for(int i = 0; i < COL_HEIGHT; i++) {
    setBothSides(i, rayToDisplay[i]);
  }
  strip.show();
}

void display_ray_image() { // takes about 242 micros
  calculate_ray(curAngle(), rayToDisplay); // takes about 30 micros (12% of time)
  displayRaySPI();
}
