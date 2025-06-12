#pragma once

#include "FastIMU.h"
#include <Wire.h>
#include <SD.h>
#include <string>
#include <algorithm>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment out this line to skip calibration at start
MPU6500 IMU(Wire2);               //Change to the name of any supported IMU!
// Other supported IMUS: MPU9255 MPU9250 MPU6886 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

const int I2C_SDA = 25;   //I2C Data pin
const int I2C_SCL = 24;   //I2c Clock pin

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

File sensorLog;


unsigned long upTimestamps[80]; // most recent 10 best geusses at when the stick was pointed upwards
unsigned long downTimestamps[80]; // most recent 10 best geusses at when the stick was pointed upwards
double yAccelHistory[80]; // last 80 samples of acceleration up/down
double yAccelDiff[40]; // elem X = yAccelHistory[X] - yAccelHistory[X-1]
double accelWeights[40];
double accelWeightTotal;
double gyroZHistory[80]; // last 80 samples degrees per second of revolution
unsigned long imuTimestamps[80];
double gyroWeightTotal;
int accelPointer = 0;
int gyroPointer = 0;

void calibrateIMU() {
  delay(3000);
  Serial.println("FastIMU calibration & data example");
  Serial.println("Keep IMU level.");

  IMU.calibrateAccelGyro(&calib);

  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);

  delay(2000);
}

void initIMU() {
  sensorLog = SD.open("sensor_log_6000_4.txt", FILE_WRITE);
  sensorLog.println("sensorLog opened");
  Wire2.begin();
  Wire2.setSDA(I2C_SDA);
  Wire2.setSCL(I2C_SCL);
  Wire2.setClock(400000); //400khz clock - max for accelerometer?

  for (int i = 0; i < 80; i++) {
    upTimestamps[i] = (unsigned long) 0;
  }
  for (int i = 0; i < 80; i++) {
    downTimestamps[i] = (unsigned long) 0;
  }

  for (int i = 0; i < 80; i++) {
    yAccelHistory[i] = (double) 0;
  }
  for (int i = 0; i < 80; i++) {
    yAccelDiff[i] = (double) 0;
  }
  for (int i = 0; i < 80; i++) {
    imuTimestamps[i] = (double) 0;
  }

  for (int i = 0; i < 80; i ++) {
    gyroZHistory[i] = (double) 0;
  }

}

void updateGyro() {
  unsigned long start = micros();
  IMU.update();
  unsigned long microdiff = micros() - start;
  Serial.print("imu update micros: ");
  Serial.println(microdiff);
  IMU.getGyro(&gyroData);
}

double getRotationalSpeed() {
  updateGyro();
  // TODO: some function of gyroX/Y/Z?
  return 0.0;
}

int loopCount = 0;

void stopRecording() {
  sensorLog.println("zzzzzzzzzzzzzzz");
  sensorLog.flush();
  sensorLog.close();
  delay(1000);
}

double rotationSpeedEstimate() {
  double sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += (gyroZHistory[gyroPointer + i] * gyroWeights[i]);
  }
  return sum / gyroWeightTotal;
}

unsigned long lastUp = micros();
unsigned long lastDown = micros() + 20;
bool orientUp = false;
bool orientDown = false;

void orient() {
  orientUp = false;
  orientDown = false;
  unsigned long startMicros = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  unsigned long sampleTime = (micros() + startMicros) / 2;
  double accY = accelData.accelY;
  int prevYPointer = (accelPointer + 39) % 40;
  int prev2YPointer = (accelPointer + 38) % 40;
  int prev3YPointer = (accelPointer + 37) % 40;
  yAccelHistory[accelPointer] = accY;
  yAccelDiff[accelPointer] = accY - yAccelHistory[prevYPointer];
  double last5DiffSum = 0;
  double last5DiffSumUnAbs = 0;
  double prev5DiffSum = 0;
  double twoPrev5DiffSum = 0;

  for (int i = -2; i <= 0; i++) {
    last5DiffSum += abs(yAccelDiff[(i + 40 + accelPointer) % 40]);
  }
  for (int i = -2; i <= 0; i++) {
    last5DiffSumUnAbs += yAccelDiff[(i + 40 + accelPointer) % 40];
  }
  for (int i = -3; i <= -1; i++) {
    prev5DiffSum += abs(yAccelDiff[(i + 40 + accelPointer) % 40]);
  }
  for (int i = -4; i <= -2; i++) {
    twoPrev5DiffSum += abs(yAccelDiff[(i + 40 + accelPointer) % 40]);
  }

  if (prev5DiffSum < last5DiffSum && prev5DiffSum > twoPrev5DiffSum) {
    if (last5DiffSumUnAbs > 0) {
      orientDown = true;
    } else {
      orientUp = true;
    }
  }

  // GYRO Z tells us which direction it's spinning, so we can account for 15 deg off

  // double prev5Sum = last5Sum + abs(yAccelDiff[(-5 + 40 + accelPointer) % 40]) - abs(yAccelDiff[accelPointer]);
  // int lessCount = 0;
  // if (accY < yAccelHistory[prevYPointer]) {
  //   lessCount += 1
  // } else {
  //   lessCount -= 1;
  // }
  // if (accY < yAccelHistory[prev2YPointer]) {
  //   lessCount += 1;
  // } else {
  //   lessCount -= 1;
  // }
  // if (yAccelHistory[prevYPointer] < yAccelHistory[prev2YPointer]) {
  //   lessCount += 1;
  // } else {
  //   lessCount -= 1;
  // }
  // if (accY < yAccelHistory[prev3YPointer]) {
  //   lessCount += 1;
  // } else {
  //   lessCount -= 1;
  // }

  // if (lessCount == 3 && lastDown > lastUp) {
  //   orientUp = true;
  //   lastUp = micros();
  // } else if (lessCount == -3 && lastDown < lastUp) {
  //   orientDown = true;
  //   lastDown = micros();
  // }

  accelPointer = (accelPointer + 1) % 40;
  gyroPointer = (gyroPointer + 1) % 100;
}

unsigned long lastGyroSampleTime = micros();
double assumedGyroAngle = 0;

void initialOrient() {
  unsigned long start = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  unsigned long microdiff = micros() - start;

  int prevYPointer = (accelPointer - 1 + 80) % 80;
  // int prev2YPointer = (accelPointer - 2 + 80) % 80;
  // int prev3YPointer = (accelPointer - 3 + 80) % 80;

  double accY = accelData.accelY;
  double gyroZ = gyroData.gyroZ;
  double prevGyroZ = 0;

  while (gyroZ < 600 || prevGyroZ < 600) {
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);
    prevGyroZ = gyroZ;
    gyroZ = gyroData.gyroZ;
    delay(300);
  } 
}

void collectData() {
  unsigned long start = micros();

  int maxY = 0;
  int minY = 0;
  double readings[600];

  for (int i = 0; i < 600; i++) {
    unsigned long loopStart = micros();
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    double accY = accelData.accelY;
    double gyroZ = gyroData.gyroZ;
    readings[i] = accY;

    unsigned long microdiff = micros() - loopStart;
    delay(5);
  }

  int n = sizeof(arr)/sizeof(arr[0]);
  sort(arr, arr + n);

  int lowAcc = 0;
  for(int i = 2; i < 6; i++) {
    lowAcc += readings[i] / 4;
  }
  int highAcc = 0;
  for(int i = 593; i < 597; i++) {
    highAcc += readings[i] / 4;
  }
}

void tryOrient() {
  unsigned long start = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  unsigned long microdiff = micros() - start;

  int prevYPointer = (accelPointer - 1 + 80) % 80;
  // int prev2YPointer = (accelPointer - 2 + 80) % 80;
  // int prev3YPointer = (accelPointer - 3 + 80) % 80;

  double accY = accelData.accelY;
  double gyroZ = gyroData.gyroZ;

  yAccelHistory[accelPointer] = accY;
  gyroZHistory[gyroPointer] = gyroZ;
  yAccelDiff[accelPointer] = accY - yAccelHistory[prevYPointer];
  imuTimestamps[accelPointer] = start;

  assumedGyroAngle += (abs(gyroZ) * 1.011 * ((start - lastGyroSampleTime) / 1000000.0)) % 360;
  bool closeTop = ((assumedGyroAngle + 15) % 360) < 26 || (assumedGyroAngle - 15) > 335;
  bool closeBot = (assumedGyroAngle < 195 && assumedGyroAngle > 165);

  if (yAccelDiff[accelPointer] > 0 && yAccelDiff[prevYPointer] > 0 && closeTop && lastDown > lastUp) {
    orientUp = true;
    lastUp = micros();
    upTimestamps[accelPointer] = lastUp;
    assumedGyroAngle = 0;
  } else if (yAccelDiff[accelPointer] < 0 && yAccelDiff[prevYPointer] < 0 && closeBot && lastDown < lastUp) {
    orientDown = true;
    lastDown = micros();
    downTimestamps[accelPointer] = lastDown;
    assumedGyroAngle = 180;
  }
  


  accelPointer = (accelPointer + 1) % 80;
  gyroPointer = (gyroPointer + 1) % 80;
}

void printStuff() {
  loopCount++;
  unsigned long start = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  unsigned long microdiff = micros() - start;
  sensorLog.print(start);
  sensorLog.print("|");
  sensorLog.print(accelData.accelX);
  sensorLog.print("|");
  sensorLog.print(accelData.accelY);
  sensorLog.print("|");
  sensorLog.print(accelData.accelZ);
  sensorLog.print("|");
  sensorLog.print(gyroData.gyroX);
  sensorLog.print("|");
  sensorLog.print(gyroData.gyroY);
  sensorLog.print("|");
  sensorLog.println(gyroData.gyroZ);
}