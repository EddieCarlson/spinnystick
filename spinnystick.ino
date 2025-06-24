#include <Snooze.h>
#include <SnoozeBlock.h>

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
SnoozeDigital digital;
SnoozeBlock config_teensy(digital);

void setup() {
  Serial.begin(9600);
  uint32_t start = millis();
  while(!Serial && (millis() - start < 2000)) { ; }
  Serial.println("hi");

  initSD(readSerial);
  // addSquares();
  initIMU();
  // calibrateIMU();
  initSPI();
  initButtons();
  setNextBrightness();
  Serial.println("started");
  ee = micros();
  digital.pinMode(13, INPUT_PULLUP, FALLING);
  digital.pinMode(14, INPUT_PULLUP, FALLING);
  digital.pinMode(15, INPUT_PULLUP, FALLING);
  digital.pinMode(16, INPUT_PULLUP, FALLING);
}

unsigned long lastLoopPrint = micros();
unsigned long lastZZ = micros();

unsigned long sampleIntervalMicros = 7000;
unsigned long lastSampleTimestamp = 0;
unsigned long lastSleepMicros = 0;
bool twoPreviouslySpinning = false;

void sleep() {
  updateButtons();
  Serial.println("going to sleep");
  Serial.flush();
  delay(500);
  Snooze.sleep( config_teensy );
  lastSleepMicros = micros();
  Serial.println("waking up");
}

void loop() {
  // unsigned long start = micros();
  checkButtonsNext();
  // Serial.println("presample");
  unsigned long curMicros = micros();
  twoPreviouslySpinning = previouslySpinning;
  if (curMicros - lastSampleTimestamp > sampleIntervalMicros) {
    sample();
    lastSampleTimestamp = curMicros;
    bool justStartedSpinningAgain = lastStartSpinningMicros < (lastStoppedSpinningMicros + 2000000);
    if (currentlySpinning && !previouslySpinning && justStartedSpinningAgain) {
      importNextImage();
    }
  }
  if (currentlySpinning || previouslySpinning || twoPreviouslySpinning) {
    displayCurImageRay();
  } else if (curMicros - lastSpinningMicros > 50000000 && curMicros - lastSleepMicros > 50000000) {
    // sleep();
  }
  // displayCardioids();
}

// ls | xargs -I {} python3 ../convert_image.py {} no_header
