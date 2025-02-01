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

const bool readSerial = false;

Bounce2::Button nextImageButton = Bounce2::Button();

void initButtons() {
  nextImageButton.attach(nextImagePin, INPUT_PULLUP);
  nextImageButton.interval(25); // millisecond debounce
  nextImageButton.setPressedState(LOW);
}

void setup() {
  Serial.begin(9600);
  uint32_t start = millis();
  while(!Serial && (millis() - start < 2000)) { ; }
  Serial.println("hi");

  initSD(readSerial);
  // addSquares();
  initSPI();
  initButtons();
  // initIMU(IMU, IMU_ADDRESS);
}

void checkButtons() {
  nextImageButton.update();
  if (nextImageButton.pressed()) {
    importNextImage();
  }
}

unsigned long lastLoopPrint = micros();

void loop() {
  unsigned long start = micros();
  checkButtons();
  displayCurImageRay();
  unsigned long duration2 = micros() - start;
  if (micros() - lastLoopPrint > 2000000) {
    Serial.println("loop");
    Serial.print("display ray image micros: ");
    Serial.println(duration2);
    lastLoopPrint = micros();
  }

  // displayCardioids();
}

