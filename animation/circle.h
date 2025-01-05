#include "../angle.h"
#include <Adafruit_DotStar.h>
#include <FastLED.h>
#include "../strip.h"

double curCirclePx = 0;
bool upCircle = true;
double circ_px_per_milli = 580.0 / 16.0;

void setCircle() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0, 0, 0);
  }

  unsigned long diff = micros() - lastMicros;

  double nextPx;
  if (upCircle) {
    nextPx = curCirclePx + (circ_px_per_milli * (double) diff / 1000.0);
  } else {
    nextPx = curCirclePx - (circ_px_per_milli * (double) diff / 1000.0);
  }
  //Serial.println(nextPx);

  if (nextPx > COL_HEIGHT) {
    nextPx = COL_HEIGHT;
    upCircle = false;
  } else if (nextPx < 0) {
    nextPx = 0;
    upCircle = true;
  }

  uint8_t nextPxInt = floor(nextPx);

  uint8_t* pixels = strip.getPixels();
  for(int i = -1; i < 2; i++) {
    uint8_t px = nextPxInt + i;

    //Serial.println(px);

    if (px > 0 && px < COL_HEIGHT) {
      strip.setPixelColor(px, CRGB::DarkOrange);
      strip.setPixelColor(COL_HEIGHT - px, CRGB::DarkOrange);
      strip.setPixelColor(200 - px, CRGB::DarkOrange);
      strip.setPixelColor(200 - (COL_HEIGHT - px), CRGB::DarkOrange);
      // double fadeByRatio = pow(1 - (abs((double) px - nextPx) / 4.0), 2);
      // fadeToBlackBy(*(pixels + (i * 3)), 1, floor((uint8_t) (fadeByRatio * 255)));
    // } else {
    //   strip.setPixelColor(px, 0);
    //   strip.setPixelColor(COL_HEIGHT - px, 0);
    //   strip.setPixelColor(200 - px, 0);
    //   strip.setPixelColor(200 - (COL_HEIGHT - px), 0);
    }
  }

  curCirclePx = nextPx;
}
