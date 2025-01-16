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

// #define SDCS 0
// #define SDMISO 1
// #define SDMOSI 26
// #define SDSCK 27

// TODO: get fucking spi working faster

// File myFile;

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
    switch ((char)myFile.peek()) {
      case ',':
      Serial.print(",");
      myFile.read();
      break;
      case '\n':
      ray++;
      px = 0;
      Serial.println();
      myFile.read();
      break;

      default:
      uint8_t r = min(255, charToHex(myFile.read()) * 16);
      uint8_t g = min(255, charToHex(myFile.read()) * 16);
      uint8_t b = min(255, charToHex(myFile.read()) * 16);

      Serial.print("(");
      Serial.print(r);
      Serial.print(",");
      Serial.print(g);
      Serial.print(",");
      Serial.print(b);
      Serial.print(")");

      rays[ray][px] = CRGB(r, g, b);

      px++;
    }
  }
  myFile.close();
}

void readFileFromSerial() {
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
  File myFile;
  String title = Serial.readStringUntil('\n', 100);
  Serial.print("title: ");
  Serial.println(title);
  myFile = SD.open(title.c_str(), FILE_WRITE);
  while(Serial.peek()) {
    String line = Serial.readStringUntil('\n', 10000);
    if (line == "ENDFILE") {
      myFile.write(line.c_str(), line.length());
      myFile.print('\n');
    }
    break;
  }
  myFile.close();
  delay(10);
}

void setup() {
  Serial.begin(9600);
  while(!Serial) { ; }

  readFileFromSerial();
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  } else {
    Serial.println("SD card init successful");
    // importImageFromSD("eddie_mandala3.txt");
  }

  SPI.begin();
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  Serial.println("spi began");

  strip.begin();
  strip.setBrightness(80); // out of...255?

  // set_image();
  // setToBlack();

  Serial.println("hi");

  // readFile();
  // initIMU(IMU, IMU_ADDRESS);
}



void setToBlack() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}

// CRGB image[image_size][image_size];

// float angle_inc = 2.0 * PI / NUM_RAYS;
// CRGB rays[NUM_RAYS][COL_HEIGHT];

float dist(float x1, float y1, float x2, float y2) {
  return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

// set reference image all black
// void reset_image() {
//   for(int row = 0; row < image_size; row++) {
//     for(int col = 0; col < image_size; col++) {
//       image[col][row] = CRGB::Black;
//     }
//   }
// }

// void set_image() {
//   reset_image();

//   for(int col = 50; col < 80; col++) {
//     for(int row = 50; row < 80; row++) {
//       if ((col < 58 || col > 72) || (row < 58 || row > 72)) {
//         image[col][row] = CRGB::Aquamarine;
//       }
//     }
//   }

//   for(int col = 180; col < 210; col++) {
//     for(int row = 180; row < 210; row++) {
//       if ((col < 188 || col > 202) || (row < 188 || row > 202)) {
//         image[col][row] = CRGB::Magenta;
//       }
//     }
//   }
// }


// for testing min time to push data over SPI
void setColorRayNothing(double angle) {
  for(int height = 0; height < COL_HEIGHT; height++) {
    strip.setPixelColor(height, 0, 1, 20);
    strip.setPixelColor(200 - height, 20, 11, 1);
  }
  strip.show();
}

// void setColorRay(double rad) {
//   for (int height = 0; height < COL_HEIGHT; height++) {
//     strip.setPixelColor(height, 0, 0, 0);
//     // TODO - direction of rad increase correct?
//     float x = (ROPE_PIXELS + height) * cos(rad); 
//     float y = (ROPE_PIXELS + height) * sin(rad);

//     int floor_x = ROPE_PIXELS + COL_HEIGHT + floor(x);
//     int floor_y = ROPE_PIXELS + COL_HEIGHT + floor(y);
//     int ceil_x = ROPE_PIXELS + COL_HEIGHT + ceil(x);
//     int ceil_y = ROPE_PIXELS + COL_HEIGHT + ceil(y);

//     //TODO - substitute floor/ceil
//     float ll_dist = 1 - dist(x, y, floor_x, floor_y);
//     float lr_dist = 1 - dist(x, y, ceil_x, floor_y);
//     float tl_dist = 1 - dist(x, y, floor_x, ceil_y);
//     float tr_dist = 1 - dist(x, y, ceil_x, ceil_y);

//     float dist_sum = ll_dist + lr_dist + tl_dist + tr_dist;
//     float adj_ll_d = ll_dist / dist_sum;
//     float adj_lr_d = lr_dist / dist_sum;
//     float adj_tl_d = tl_dist / dist_sum;
//     float adj_tr_d = tr_dist / dist_sum;

//     // Serial.print("(");
//     // Serial.print(floor_x);
//     // Serial.print(", ");
//     // Serial.print(floor_y);
//     // Serial.print("), ");

//     CRGB ll_color = image[floor_x][floor_y];
//     CRGB lr_color = image[ceil_x][floor_y];
//     CRGB tl_color = image[floor_x][ceil_y];
//     CRGB tr_color = image[ceil_x][ceil_y];

//     // TODO - uhh lol i think this maybe works? maybe round and bound instead of floor?
//     uint8_t blend_r = floor(sqrt(pow(ll_color[0] * adj_ll_d, 2) + pow(lr_color[0] * adj_lr_d, 2) + pow(tl_color[0] * adj_tl_d, 2) + pow(tr_color[0] * adj_tr_d, 2)));
//     uint8_t blend_g = floor(sqrt(pow(ll_color[1] * adj_ll_d, 2) + pow(lr_color[1] * adj_lr_d, 2) + pow(tl_color[1] * adj_tl_d, 2) + pow(tr_color[1] * adj_tr_d, 2)));
//     uint8_t blend_b = floor(sqrt(pow(ll_color[2] * adj_ll_d, 2) + pow(lr_color[2] * adj_lr_d, 2) + pow(tl_color[2] * adj_tl_d, 2) + pow(tr_color[2] * adj_tr_d, 2)));
//     // Serial.println(blend_b);
//     if (blend_r + blend_g + blend_b > 4) { 
//       strip.setPixelColor(height, blend_r, blend_g, blend_b);
//       strip.setPixelColor(200 - height, blend_r, blend_g, blend_b);
//     } else {
//       strip.setPixelColor(height, 0, 0, 0);
//       strip.setPixelColor(200 - height, 0, 0, 0);
//     }
//   }
// }

unsigned long lastLoopPrint = micros();

void loop() {
  unsigned long start = micros();
  display_ray_image();
  unsigned long duration2 = micros() - start;
  bool print = micros() - lastLoopPrint > 1000000;
  if (micros() - lastLoopPrint > 1000000) {
    Serial.println("loop");
    Serial.print("display ray image micros: ");
    Serial.println(duration2);
    lastLoopPrint = micros();
  }




  // displayCardioids();
  // delayMicroseconds(500);
  //gyro();
   //setColorRay(curAngle());
  //  Serial.println("loop");






    // strip.setPixelColor(0, 100, 0, 200);
    // strip.setPixelColor(5, 100, 0, 200);
    // strip.setPixelColor(15, 100, 0, 200);
    // unsigned long startMicros = micros();
    // strip.show();
    // unsigned long duration = micros() - startMicros;
    // if (print) {
    //   Serial.println(duration);
    // }
    // delay(1000);
}

