#pragma once

#include <Adafruit_DotStar.h>
#include "common.h"
#include <FastLED.h>

#define DATAPIN    11
#define CLOCKPIN   13
#define BRIGHTNESS 8 // 0 to 31

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

bool setBothSides(int px, uint8_t r, uint8_t g, uint8_t b) {
  bool settable = px >= 0 && px < COL_HEIGHT;
  if (settable) {
    strip.setPixelColor(px, r, g, b);
    strip.setPixelColor(200 - px, r, g, b);
  }
  return settable;
}

bool setBothSides(int px, uint32_t hex) {
  return setBothSides(px, ((uint8_t) ((hex >> 16) & 0xFF)), ((uint8_t) ((hex >> 8) & 0xFF)), ((uint8_t) (hex & 0xFF)));
}