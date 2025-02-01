#pragma once

#include <arduino.h>
#include <Adafruit_DotStar.h>
#include "common.h"
#include <FastLED.h>
#include <math.h>

#define DATAPIN    26
#define CLOCKPIN   27
#define BRIGHTNESS 31 // 0 to 31

const double maxBrightPct = 0.23;
const double minBrightFactorOnly = 0.051;

double brightness_factor = 0.07; // 0 - 1

uint32_t brightness32 = 0xFF000000; // keep at FF if possible

int curBrightnessIndex = 2;

class Brightness {
  public:
    double rgb_factor;
    uint32_t brightness32;

    Brightness(double pctBright) {
      pctBright = min(max(0, pctBright), maxBrightPct);
      if (pctBright > minBrightFactorOnly) {
        brightness32 = 0xFF000000;
        rgb_factor = pctBright;
      } else {
        brightness32 = 0xF0000000;
        rgb_factor = pctBright * 2.0;
      }
    }
};

Brightness brightnesses[6] = {Brightness(0.03), Brightness(0.05), Brightness(0.08), Brightness(0.12), Brightness(0.16), Brightness(0.21)};

CRGB pixels[COL_HEIGHT];

inline uint32_t getFrame(uint8_t r, uint8_t g, uint8_t b) {
  return brightness32 | ((uint32_t) (b << 16) & 0xFF0000) | ((uint32_t) ((g << 8) & 0xFF00)) | ((uint32_t) ((r & 0xFF)));
}

inline uint32_t getFrame(CRGB color) {
  return getFrame(color.r, color.g, color.b);
}

void clearPixels() {
  for(int i = 0; i < COL_HEIGHT; i++) {
    pixels[i] = CRGB::Black;
  }
}

void displayPixels() {
  SPI1.transfer32((uint32_t) 0);
  for(int i = 0; i < COL_HEIGHT; i++) {
    SPI1.transfer32(getFrame(pixels[i]));
  }
  for(int i = 0; i < NUMPIXELS - (COL_HEIGHT * 2); i++) {
    SPI1.transfer32(brightness32);
  }
  for(int i = COL_HEIGHT - 1; i >= 0; i--) {
    SPI1.transfer32(getFrame(pixels[i]));
  }
  for (uint16_t i = 0; i < (NUMPIXELS + 14)/64; i++) {
    SPI1.transfer32(0);
  }
  return;
}

void initSPI() {
  SPI1.begin();
  SPI1.setMOSI(DATAPIN);
  SPI1.setSCK(CLOCKPIN);
  SPI1.beginTransaction(SPISettings(32000000, MSBFIRST, SPI_MODE0));

  Serial.print("SPI enabled on datapin: ");
  Serial.print(DATAPIN);
  Serial.print(", clockpin: ");
  Serial.println(CLOCKPIN);
}

bool setPixels(int px, uint8_t r, uint8_t g, uint8_t b) {
  bool settable = px >= 0 && px < COL_HEIGHT;
  if (settable) {
    pixels[px] = CRGB(r, g, b);
  }
  return settable;
}

void setBrightness(Brightness b) {
  brightness_factor = b.rgb_factor;
  brightness32 = b.brightness32;
}

void setNextBrightness() {
  setBrightness(brightnesses[curBrightnessIndex]);
  curBrightnessIndex = (curBrightnessIndex + 1) % 6;
}