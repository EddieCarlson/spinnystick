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

const bool readSerial = false;
const String defaultSDImageFilename = "eddie_mandala3.txt";

const uint8_t interruptPin = 39;

void waitForSerial() {
  uint32_t start = millis();
  while(!Serial && (millis() - start < 2000)) { ; }
}

void hi() {
  Serial.println("hi");
}

void setup() {
  Serial.begin(9600);
  waitForSerial();

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  } else {
    Serial.println("SD card init successful");
    // String filename = defaultSDImageFilename;
    // if (readSerial) {
    //   filename = readFileFromSerial();
    // }
    // importImageFromSD(filename);
  }

  // Serial.println(selectImageNameByIndex());
  // listAnimations();

  SPI.begin();
  SPI.setMOSI(DATAPIN);
  SPI.setSCK(CLOCKPIN);
  SPI.beginTransaction(SPISettings(32000000, MSBFIRST, SPI_MODE0));
  SPI.setMOSI(DATAPIN);
  SPI.setSCK(CLOCKPIN);
  Serial.println("spi began");

  Serial.println("hi");

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), importNextImage, FALLING);
  interrupts();

  importNextImage();

  // initIMU(IMU, IMU_ADDRESS);
}



void setToBlack() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0, 0, 0);  
  }
  strip.show();
}

float dist(float x1, float y1, float x2, float y2) {
  return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}


// for testing min time to push data over SPI
void setColorRayNothing(double angle) {
  for(int height = 0; height < COL_HEIGHT; height++) {
    strip.setPixelColor(height, 0, 1, 20);
    strip.setPixelColor(200 - height, 20, 11, 1);
  }
  strip.show();
}

unsigned long lastLoopPrint = micros();

void loop() {
  unsigned long start = micros();
  display_ray_image();
  unsigned long duration2 = micros() - start;
  if (micros() - lastLoopPrint > 2000000) {
    Serial.println("loop");
    Serial.print("display ray image micros: ");
    Serial.println(duration2);
    lastLoopPrint = micros();
  }

  // displayCardioids();
}

