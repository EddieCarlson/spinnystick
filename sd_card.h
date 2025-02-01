#pragma once

#include <SD.h>
#include <FastLED.h>
#include "common.h"
#include "animation/raydisplay.h"

volatile bool changeImage = false;
int imageIndex = 0;
int maxImageIndex = -1;

String selectImageNameByIndex(bool printNames) {
  File imagesRoot = SD.open("/images");
  imagesRoot.rewindDirectory();
  File f = File();
  String name;
  int imageCount = -1;
  while(imageCount < imageIndex) {
    if (f) {
      f.close();
      delay(20);
    }

    f = imagesRoot.openNextFile();
    if (!f) { // no files remaining - cycle back to first file
      if (maxImageIndex == -1) {
        maxImageIndex = imageIndex - 1;
      }
      imageIndex = 0;
      imagesRoot.close();
      delay(50);
      return selectImageNameByIndex(false);
    }
    name = String(f.name());
    if (name.endsWith(".rayb")) {
      if (printNames) {
        Serial.println(name);
      }
      imageCount++;
    }
  }
  if (f) {
    f.close();
    delay(20);
  }
  imagesRoot.close();
  delay(20);
  return "images/" + name;
}

String selectImageNameByIndex() {
  return selectImageNameByIndex(false);
}

void listAllImages() {
  int curImageIndex = imageIndex;
  imageIndex = 200;
  Serial.println("images: ");
  selectImageNameByIndex(true);
  imageIndex = curImageIndex;
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
  Serial.print("importing image: ");
  Serial.println(name);

  File myFile = SD.open(name.c_str());
  char colorHex[3];
  for(int ray = 0; ray < NUM_RAYS; ray++) {
    for(int px = 0; px < COL_HEIGHT; px++) {
      myFile.readBytes(colorHex, 3);
      uint8_t r = colorHex[0];
      uint8_t g = colorHex[1];
      uint8_t b = colorHex[2];
      imageRays[ray][px] = CRGB(r, g, b);
    }
  }
  myFile.close();
  delay(8);
}

String readFileFromSerial() {
  uint32_t startMillis = millis();
  while(!Serial.available()) {
    if (millis() - startMillis < 300000) {
      delay(1000);
    } else {
      Serial.println("did not receive serial input after 5 minutes");
      return "failure";
    }
  }
  String title = "images/" + Serial.readStringUntil('|', 200);
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
  importImageFromSD(selectImageNameByIndex());
  imageIndex++;
}

void importPrevImage() {
  int prevImageIndex = imageIndex - 2;
  if (prevImageIndex < 0) {
    prevImageIndex = maxImageIndex;
  }
  importNextImage();
}

void initSD(bool readSerial) {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  } else {
    Serial.println("SD card init successful");
    if (readSerial) {
      String filename = readFileFromSerial();
      importImageFromSD(filename);
    } else {
      // importNextImage();
      importImageFromSD("images/butterfly.rayb");
    }
  }

  delay(100);

  listAllImages();
}

void setNextImageBool() {
  changeImage = true;
}
