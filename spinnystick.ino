#include <arduino.h>
#include <SPI.h>
#include <FastLED.h>

#include "common.h"
#include "strip.h"
#include "angle.h"
#include "imu_init.h"
#include "animation/circle.h"
#include "animation/raydisplay.h"
#include "animation/polar_calc.h"
#include <Adafruit_DotStar.h>
#include <ctype.h>

#include "sd_card.h"
#include "animation/shapes.h"
#include <Bounce2.h>
#include "buttons.h"
#include "imu_init.h"

const bool readSerial = false;
unsigned long ee = micros();
void setup() {
  Serial.begin(9600);
  uint32_t start = millis();
  while(!Serial && (millis() - start < 2000)) { ; }
  Serial.println("hi");

  // initSD(readSerial);
  // addSquares();
  initIMU();
  // calibrateIMU();
  initSPI();
  initButtons();
  setNextBrightness();
  setNextPeriod();
  // delay(15000);
  Serial.println("started");
  // setAll(CRGB(20,0,0));
  setTop(CRGB(0,0,30));
  // setAll(CRGB(0,0,0));
  // displayCurImageRay();
  ee = micros();
}

unsigned long lastLoopPrint = micros();
unsigned long lastZZ = micros();

unsigned long sampleIntervalMicros = 10000;
unsigned long lastSampleTimestamp = 0;

void loop() {
  // unsigned long start = micros();
  // checkButtonsNext();
  // Serial.println("presample");
  unsigned long curMicros = micros();
  if (curMicros - lastSampleTimestamp > sampleIntervalMicros) {
    sample();
    lastSampleTimestamp = curMicros;
  }
  displayCurImageRay();
  // Serial.println(duration);
  // updateIMU();
  // Serial.println("postsample");
  // if (currentlySpinning) {
    // setAll(CRGB(20,0,0));
  //   displayCurImageRay();
  // } else {
  //   setAll(CRGB(0,0,0));
  //   displayCurImageRay();
  // }

  // if (oldCurrentSpin && !currentlySpinning) {
  //   sensorLog.flush();
  //   sensorLog.close();
  //   delay(3000);
  // }

  // Serial.println("preprint");

  // printStuff();

  // Serial.println("");
  // if (imageInitialized) {
  //   displayCurImageRay();
  // }
  // unsigned long duration2 = micros() - start;
  // while (micros() - start < 8000) { ; }

  // displayCardioids();
}

// ls | xargs -I {} python3 ../convert_image.py {} no_header
