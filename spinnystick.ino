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

const uint8_t interruptPin = 40;

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
  Serial.println("hi");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  } else {
    Serial.println("SD card init successful");
    if (readSerial) {
      String filename = readFileFromSerial();
      importImageFromSD(filename);
    } else {
      importNextImage();
    }
  }

  delay(1000);

  listAllImages();

  SPI1.begin();
  SPI1.setMOSI(DATAPIN);
  SPI1.setSCK(CLOCKPIN);
  SPI1.beginTransaction(SPISettings(32000000, MSBFIRST, SPI_MODE0));

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), setNextImageBool, FALLING);
  interrupts();

  // initIMU(IMU, IMU_ADDRESS);
}

void setNextImageBool() {
  changeImage = true;
}

unsigned long lastLoopPrint = micros();

void loop() {
  unsigned long start = micros();
  if (changeImage) {
    importNextImage();
    changeImage = false;
  }
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

