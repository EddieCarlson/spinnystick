#pragma once

#include "FastIMU.h"
#include <Wire.h>
#include <SD.h>
#include <string>

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


#define orientationHistorySize 5
#define gyroHistorySize 60
#define smoothingWindow 5

int orientationHistoryPointer = 0;
int gyroHistoryPointer = 0;
double revTrackingAngle = 0;
double curAngleEstimate = -1;
int revTrackingStartPointer = 0;
int upIndex = 0;
int upTimestamp = 0;
bool newUpIndex = false;

double minDegreesPerSec = 800;

double yAccelHistory[gyroHistorySize]; // most recent samples of acceleration up/down
double gyroZHistory[gyroHistorySize]; // most recent samples degrees per second of revolution
double smoothedGyroHistory[gyroHistorySize];
unsigned long timestamps[gyroHistorySize];

unsigned long upTimestamps[orientationHistorySize]; // most recent best geusses at when the stick was pointed upwards
unsigned long downTimestamps[orientationHistorySize]; // most recent best geusses at when the stick was pointed upwards
double yAccelDiff[gyroHistorySize]; // elem X = yAccelHistory[X] - yAccelHistory[X-1]
double accelWeights[gyroHistorySize];
double accelWeightTotal;
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

int unboundedGyroPointer(int unboundedIndex) {
  return (unboundedIndex + gyroHistorySize) % gyroHistorySize;
}

int gyroAt(int unboundedIndex) {
  return gyroZHistory[unboundedGyroPointer(unboundedIndex)];
}

int gyroPointerFor(int offset) {
  return (gyroHistoryPointer + gyroHistorySize + offset) % gyroHistorySize;
}

double prevGyro(int offset) {
  return gyroZHistory[gyroPointerFor(offset)];
}

unsigned long microsBetween(int startIndex, int endIndex) {
  return timestamps[unboundedGyroPointer(endIndex)] - timestamps[unboundedGyroPointer(startIndex)];
}

unsigned long timeSince(int offset) {
  return timestamps[gyroHistoryPointer] - timestamps[gyroPointerFor(offset)];
}

void update() {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  gyroHistoryPointer = (gyroHistoryPointer + 1) % gyroHistorySize;
  yAccelHistory[gyroHistoryPointer] = accelData.accY;
  gyroZHistory[gyroHistoryPointer] = gyroData.gyroZ;
  timestamps[gyroHistoryPointer] = micros();

  // TODO: accel at
  double smoothedInit = (gyroAt(gyroHistoryPointer) * 0.6) + (gyroAt(gyroHistoryPointer - 1) * 0.15) + (gyroAt(gyroHistoryPointer - 2) * 0.05);
  smoothedGyroHistory[gyroHistoryPointer] = smoothedInit;
  smoothedGyroHistory[unboundedGyroPointer(gyroHistoryPointer - 1)] += gyroAt(gyroHistoryPointer) * 0.15;
  smoothedGyroHistory[unboundedGyroPointer(gyroHistoryPointer - 2)] += gyroAt(gyroHistoryPointer) * 0.05;

  // TODO: clear others at +3 as well?
  smoothedGyroHistory[unboundedGyroPointer(gyroHistory + 3)] = 0;
}

bool isFastEnough() {
  bool spinningFast = true
  for (int i = -5; i <= 0; i++) {
    spinningFast = spinningFast && (prevGyro(i) > minDegreesPerSec);
  }
  return spinningFast;
}

int boundedGyroIndex(int anyIndex) {
  int boundedIndex = anyIndex % gyroHistorySize;
  if (boundedIndex < 0) {
    boundedIndex = boundedIndex + gyroHistorySize;
  }
  return boundedIndex;
}

// TODO: use everywhere
double getGyroZ(int anyIndex) {
  return gyroZHistory[boundedGyroIndex(anyIndex)];
}

double degreesTraveledForIndex(int start) {
  double avgZ = (getGyroZ(start) + getGyroZ(start - 1)) / 2.0
  return avgZ * microsBetween(start, start - 1)
}

bool updateRevTracker() {
  double degreesSinceLastTimestamp = gyroZHistory[gyroHistoryPointer] * timeSince(-1) / 1000000;
  // % 400 (instead of 360) to account for gyro integration not being perfect, to guarantee a high+low?
  double newAngle = (revTrackingAngle + degreesSinceLastTimestamp) % 360; 
  bool hasRevolved = newAngle < revTrackingAngle;
  revTrackingAngle = newAngle;
  return hasRevolved;
}

int upIndexInLastRev() {
  double largestSmoothed = 0;
  int largestSmoothedIndex = revTrackingStartPointer;
  for (int i = revTrackingStartPointer; i != gyroHistoryPointer - 2; i = (i + 1) % gyroHistorySize) {
    double smoothed = abs(smoothedGyroHistory[i]);
    if (smoothed > largestSmoothed) {
      largestSmoothed = smoothed;
      largestSmoothedIndex = i;
    }
  }

  return largestSmoothedIndex;
}

double estimateAngleFromLastUp() {
  // obob?
  double estimate = 0;
  for (int i = upIndex; i != gyroHistoryPointer + 1; i = (i + 1) % 60) {
    estimate += degreesTraveledForIndex(i);
  }
  return estimate;
}

int successiveLargeDiscrepancies = 0;

double updateAngleEstimate(bool revComplete) {
  bool notFirstRev = curAngleEstimate > -0.5;
  if (revComplete) {
    double estimatedFromUp = estimateAngleFromLastUp();
    bool largeDiscrepancy = notFirstRev && (abs(curAngleEstimate - estimatedFromUp) > (50 + (successiveLargeDiscrepancies * 35)));
    if (largeDiscrepancy) {
      successiveLargeDiscrepancies += 1;
    } else {
      successiveLargeDiscrepancies = 0;
    }
    if (!largeDiscrepancy) {
      curAngleEstimate = estimatedFromUp;
    }
  } else if (notFirstRev) {
    curAngleEstimate += degreesTraveledForIndex(gyroHistoryPointer);
    curAngleEstimate = curAngleEstimate % 360;
  }
}

void sample() {
  update();
  if (!isFastEnough()) {
    revTrackingAngle = 0;
    revTrackingStartPointer = gyroHistoryPointer;
    curAngleEstimate = -1;
    return
  }
  bool revComplete = updateRevTracker();
  if (revComplete) {
    upIndex = smoothGyro();
    newUpIndex = true;
    upTimestamp = timestamps[upIndex];
    revTrackingStartPointer = gyroHistoryPointer;
  }
  updateAngleEstimate(revComplete);
}

// TODO: only display image when stick has had a gyroZ > 400 in the last second


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