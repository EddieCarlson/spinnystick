#pragma once

#include <SD.h>
#include <FastLED.h>
#include "common.h"
#include "animation/raydisplay.h"

volatile bool changeAnimations = false;
int animationIndex = 0;

String selectImageNameByIndex() {
  File root = SD.open("/");
  File f = File();
  String name;
  int animationCount = -1;
  while(animationCount < animationIndex) {
    if (f) { f.close(); }

    f = root.openNextFile();
    if (!f) { // no files remaining - cycle back to first file
      animationIndex = 0;
      return selectImageNameByIndex();
    }
    name = String(f.name());
    if (name.endsWith(".txt")) { // currently counting all root level .txt files as images
      animationCount++;
    }
  }
  if (f) { f.close(); }
  root.close();
  return name;
}

uint8_t charToHex(char hexChar) {
  if (hexChar >= '0' && hexChar <= '9') {
      return hexChar - '0';
  } else if (hexChar >= 'A' && hexChar <= 'F') {
      return hexChar - 'A' + 10;
  } else if (hexChar >= 'a' && hexChar <= 'f') {
      return hexChar - 'a' + 10;
  } else {
    // Serial.print("bad hex-char conversion from: ");
    // Serial.println(hexChar);
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

void importNextImage() {
  Serial.println("import next image: ");
  String nextImage = selectImageNameByIndex();
  Serial.println(nextImage);
  importImageFromSD(nextImage);
  animationIndex++;
}
