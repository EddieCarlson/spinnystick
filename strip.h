#pragma once

#include <Adafruit_DotStar.h>
#include "common.h"
#include <FastLED.h>

#define DATAPIN    26
#define CLOCKPIN   27
#define BRIGHTNESS 31 // 0 to 31

const double brightness_factor = 0.08; // 0 - 1

// TODO: raise this to FF if possible
const uint32_t brightness32 = 0xFF000000;

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