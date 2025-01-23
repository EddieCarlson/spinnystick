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

#include <SD.h>

uint8_t charToHex(char hexChar) {
  if (hexChar >= '0' && hexChar <= '9') {
      return hexChar - '0';
  } else if (hexChar >= 'A' && hexChar <= 'F') {
      return hexChar - 'A' + 10;
  } else if (hexChar >= 'a' && hexChar <= 'f') {
      return hexChar - 'a' + 10;
  } else {
    Serial.print("bad hex-char conversion from: ");
    Serial.println(hexChar);
    return 0;
  }
}

void importImageFromSD(String name) {
  File myFile = SD.open(name.c_str());
  int ray = 0;
  int px = 0;
  while (myFile.available()) {
    if (ray >= NUM_RAYS) {
      break;
    }
    switch ((char)myFile.peek()) {
      case ',':
      myFile.read();
      break;
      case '|':
      ray++;
      px = 0;
      myFile.read();
      break;

      default:
      int ri = myFile.read(); 
      int gi = myFile.read(); 
      int bi = myFile.read(); 

      uint8_t r = min(255, charToHex(ri) * 16);
      uint8_t g = min(255, charToHex(gi) * 16);
      uint8_t b = min(255, charToHex(bi) * 16);

      rays[ray][px] = CRGB(r, g, b);

      px++;
    }
  }
  myFile.close();
}

String readFileFromSerial() {
  delay(1000);
  uint32_t startMillis = millis();
  Serial.println("waiting for serial input");
  while(!Serial.available()) {
    if (millis() - startMillis < 300000) {
      delay(1000);
    } else {
      Serial.println("serial input received");
      return;
    }
  }
  String title = Serial.readStringUntil('|', 200);
  File myFile = SD.open(title.c_str(), FILE_WRITE);
  while(true) {
    myFile.write(Serial.read());
    if (!Serial.available()) {
      delay(10);
      if (!Serial.available()) {
        break;
      }
    }
  }
  Serial.println("finished writing file");
  delay(50);
  myFile.close();
  delay(20);
  return title;
}

const bool readSerial = false;
const String defaultSDImageFilename = "eddie_mandala5.txt";

void waitForSerial() {
  uint32_t start = millis();
  while(!Serial && (millis() - start < 2000)) { ; }
}

void setup() {
  Serial.begin(9600);
  waitForSerial();

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  } else {
    Serial.println("SD card init successful");
    String filename = defaultSDImageFilename;
    if (readSerial) {
      filename = readFileFromSerial();
    }
    importImageFromSD(filename);
  }

  SPI.begin();
  SPI.setMOSI(DATAPIN);
  SPI.setSCK(CLOCKPIN);
  SPI.beginTransaction(SPISettings(32000000, MSBFIRST, SPI_MODE0));
  SPI.setMOSI(DATAPIN);
  SPI.setSCK(CLOCKPIN);
  Serial.println("spi began");

  Serial.println("hi");

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

