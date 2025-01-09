#include <SPI.h>
#include <FastLED.h>

#include "angle.h"
#include "strip.h"
#include "animation/circle.h"
#include "imu_init.h"

#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment out this line to skip calibration at start
MPU6500 IMU;               //Change to the name of any supported IMU!
// Other supported IMUS: MPU9255 MPU9250 MPU6886 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

// TODO: get fucking spi working faster
void setup() {
  SPI.begin();
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));

  Serial.begin(9600);

  strip.begin();
  strip.setBrightness(70); // out of...255?

  set_image();
  setToBlack();

  while(!Serial) { ; }

  Serial.println("hi");
  initIMU(IMU, IMU_ADDRESS);
}

void setToBlack() {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}

const int image_size = (ROPE_PIXELS + COL_HEIGHT) * 2; // 278x278 px

CRGB image[image_size][image_size];

// float angle_inc = 2.0 * PI / NUM_RAYS;
// CRGB rays[NUM_RAYS][COL_HEIGHT];

float dist(float x1, float y1, float x2, float y2) {
  return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

// set reference image all black
void reset_image() {
  for(int row = 0; row < image_size; row++) {
    for(int col = 0; col < image_size; col++) {
      image[col][row] = CRGB::Black;
    }
  }
}

void set_image() {
  reset_image();

  for(int col = 50; col < 80; col++) {
    for(int row = 50; row < 80; row++) {
      if ((col < 58 || col > 72) || (row < 58 || row > 72)) {
        image[col][row] = CRGB::Aquamarine;
      }
    }
  }

  for(int col = 180; col < 210; col++) {
    for(int row = 180; row < 210; row++) {
      if ((col < 188 || col > 202) || (row < 188 || row > 202)) {
        image[col][row] = CRGB::Magenta;
      }
    }
  }
}


// for testing min time to push data over SPI
void setColorRayNothing(double angle) {
  for(int height = 0; height < COL_HEIGHT; height++) {
    strip.setPixelColor(height, 0, 1, 20);
    strip.setPixelColor(200 - height, 20, 11, 1);
  }
  strip.show();
}

void setColorRay(double rad) {
  for (int height = 0; height < COL_HEIGHT; height++) {
    strip.setPixelColor(height, 0, 0, 0);
    // TODO - direction of rad increase correct?
    float x = (ROPE_PIXELS + height) * cos(rad); 
    float y = (ROPE_PIXELS + height) * sin(rad);

    int floor_x = ROPE_PIXELS + COL_HEIGHT + floor(x);
    int floor_y = ROPE_PIXELS + COL_HEIGHT + floor(y);
    int ceil_x = ROPE_PIXELS + COL_HEIGHT + ceil(x);
    int ceil_y = ROPE_PIXELS + COL_HEIGHT + ceil(y);

    //TODO - substitute floor/ceil
    float ll_dist = 1 - dist(x, y, floor_x, floor_y);
    float lr_dist = 1 - dist(x, y, ceil_x, floor_y);
    float tl_dist = 1 - dist(x, y, floor_x, ceil_y);
    float tr_dist = 1 - dist(x, y, ceil_x, ceil_y);

    float dist_sum = ll_dist + lr_dist + tl_dist + tr_dist;
    float adj_ll_d = ll_dist / dist_sum;
    float adj_lr_d = lr_dist / dist_sum;
    float adj_tl_d = tl_dist / dist_sum;
    float adj_tr_d = tr_dist / dist_sum;

    // Serial.print("(");
    // Serial.print(floor_x);
    // Serial.print(", ");
    // Serial.print(floor_y);
    // Serial.print("), ");

    CRGB ll_color = image[floor_x][floor_y];
    CRGB lr_color = image[ceil_x][floor_y];
    CRGB tl_color = image[floor_x][ceil_y];
    CRGB tr_color = image[ceil_x][ceil_y];

    // TODO - uhh lol i think this maybe works? maybe round and bound instead of floor?
    uint8_t blend_r = floor(sqrt(pow(ll_color[0] * adj_ll_d, 2) + pow(lr_color[0] * adj_lr_d, 2) + pow(tl_color[0] * adj_tl_d, 2) + pow(tr_color[0] * adj_tr_d, 2)));
    uint8_t blend_g = floor(sqrt(pow(ll_color[1] * adj_ll_d, 2) + pow(lr_color[1] * adj_lr_d, 2) + pow(tl_color[1] * adj_tl_d, 2) + pow(tr_color[1] * adj_tr_d, 2)));
    uint8_t blend_b = floor(sqrt(pow(ll_color[2] * adj_ll_d, 2) + pow(lr_color[2] * adj_lr_d, 2) + pow(tl_color[2] * adj_tl_d, 2) + pow(tr_color[2] * adj_tr_d, 2)));
    // Serial.println(blend_b);
    if (blend_r + blend_g + blend_b > 4) { 
      strip.setPixelColor(height, blend_r, blend_g, blend_b);
      strip.setPixelColor(200 - height, blend_r, blend_g, blend_b);
    } else {
      strip.setPixelColor(height, 0, 0, 0);
      strip.setPixelColor(200 - height, 0, 0, 0);
    }
  }
}

void loop() {
  //gyro();
   //setColorRay(curAngle());
    curAngle();
    setCircle();
    // strip.show();
    // curAngle();
    strip.show();


  // SPI.endTransaction();
  // for (int i = 0; i < 630; i++) { 
  //   // Serial.println();
  //   // Serial.println();
  //   // Serial.print("angle: ");
  //   // Serial.println(i);
  //   unsigned long start = micros();
  //   setColorRay(i / 1.75);
  //   unsigned long duration = micros() - start;


  //   delayMicroseconds(1050 - duration);
  //   // Serial.println(duration);
  // }
  // delay(10000);

  // unsigned long duration = micros() - time;
  // sprintf(buf, "duration: %lu", duration);
  // Serial.println(buf);




  // Serial.print("strip refresh rate: ");

  // unsigned long time = micros();
  // for (int t = 0; t < times; t++) {
  //   for (int i = 0; i < TEST_PIXELS; i++) {
  //     strip.setPixelColor(i, 0, 8, 20);
  //   }
  //   strip.show();
  //   // delay(3);

  //   for (int i = 0; i < TEST_PIXELS; i++) {
  //     strip.setPixelColor(i, 20, 0, 8);
  //   }
  //   strip.show();
  //   // delay(3);
  // }
  // unsigned long duration = micros() - time;
  // double timesd = times;
  // double refresh_rate = duration / timesd / 2.0 / 1000.0;
  // sprintf(buf, "%f", refresh_rate);
  // Serial.println(buf);
}
/*
void gyro() {
  for(;;) {
  unsigned long start = micros();
  IMU.update();
  unsigned long microdiff = micros() - start;
  Serial.print("micros: ");
  Serial.println(microdiff);
  IMU.getAccel(&accelData);
  Serial.print("Accel - x:");
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print("y:");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print("z:");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  IMU.getGyro(&gyroData);
  Serial.print("Gyro - x:");
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print("y:");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print("z:");
  Serial.print(gyroData.gyroZ);
  if (IMU.hasTemperature()) {
      Serial.print("\t");
    Serial.print("Temp:");
      Serial.println(IMU.getTemp());
  }
  delay(1500);
  }
}
*/
