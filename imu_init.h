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
int historyCurIndex = 0;
double revTrackingAngle = 0;
double curAngleEstimate = -1;
int revTrackingStartPointer = 0;
int upIndex = 0;
int upTimestamp = 0;
bool newUpIndex = false;

double minDegreesPerSec = 800;

double accelHistory[historySize]; // most recent samples of acceleration up/down
double gyroHistory[historySize]; // most recent samples degrees per second of revolution
double smoothedAccelHistory[historySize];
unsigned long timestamps[historySize];

unsigned long upTimestamps[orientationHistorySize]; // most recent best geusses at when the stick was pointed upwards
unsigned long downTimestamps[orientationHistorySize]; // most recent best geusses at when the stick was pointed upwards
double yAccelDiff[historySize]; // elem X = yAccelHistory[X] - yAccelHistory[X-1]
double accelWeights[historySize];
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

  for (int i = 0; i < historySize; i++) {
    gyroHistory[i] = (double) 0;
  }
  for (int i = 0; i < historySize; i++) {
    accelHistory[i] = (double) 0;
  }
  for (int i = 0; i < historySize; i++) {
    timestamps[i] = (unsigned long) 0;
  }
}

int loopCount = 0;

void stopRecording() {
  sensorLog.println("zzzzzzzzzzzzzzz");
  sensorLog.flush();
  sensorLog.close();
  delay(1000);
}

int boundedHistoryIndex(int anyIndex) {
  int boundedIndex = anyIndex % historySize;
  if (boundedIndex < 0) {
    boundedIndex = boundedIndex + historySize;
  }
  return boundedIndex;
}

double getGyro(int anyIndex) { return gyroHistory[boundedHistoryIndex(anyIndex)]; }

double getCurGyro(int offset) { return getGyro(historyCurIndex + offset); }

double getAccel(int anyIndex) { return accelHistory[boundedHistoryIndex(anyIndex)]; }

double getCurAccel(int offset) { return getAccel(historyCurIndex + offset); }

double getTimestamp(int anyIndex) { return timestamps[boundedHistoryIndex(anyIndex)]; }

double getCurTimestamp(int offset) { return getTimestamp(historyCurIndex + offset); }

unsigned long microsBetween(int startIndex, int endIndex) {
  return timestamps[boundedHistoryIndex(endIndex)] - timestamps[boundedHistoryIndex(startIndex)];
}

unsigned long timeSince(int offset) {
  return timestamps[historyCurIndex] - timestamps[getCurTimestamp(offset)];
}

void update() {
  historyCurIndex = (historyCurIndex + 1) % historySize;

  unsigned long startMicros = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  unsigned long updateMicros = (micros() + startMicros) / 2;
  timestamps[historyCurIndex] = updateMicros;

  accelHistory[historyCurIndex] = accelData.accY;
  double gyroZX = pow(pow(gyroData.gyroZ, 2) + pow(gyroData.gyroX, 2), 0.5);
  gyroHistory[historyCurIndex] = gyroZX;

  // smoothedAccelHistory[i] will equal: weights [0.08, 0.12, 0.6, 0.12, 0.08] * raw accel values centered on curHistoryIndex
  // smoothedAccelHistory[i] will only be complete once the curHistoryIndex = i + 2
  double smoothedInit = (accelData.accY * 0.6) + (accelAt(historyCurIndex - 1) * 0.12) + (accelAt(historyCurIndex - 2) * 0.08);
  smoothedAccelHistory[historyCurIndex] = smoothedInit;
  smoothedAccelHistory[boundedHistoryIndex(historyCurIndex - 1)] += accelData.accY * 0.12;
  smoothedAccelHistory[boundedHistoryIndex(historyCurIndex - 2)] += accelData.accY * 0.08;

  gyroHistory[boundedHistoryIndex(historyCurIndex + 3)] = 0;
  accelHistory[boundedHistoryIndex(historyCurIndex + 3)] = 0;
  smoothedAccelHistory[boundedHistoryIndex(historyCurIndex + 3)] = 0;
}

bool isSpinning() {
  bool spinningFast = true;
  for (int i = -5; i <= 0; i++) {
    spinningFast = spinningFast && (prevGyro(i) > minDegreesPerSec);
  }
  return spinningFast;
}

double degreesTraveledForIndex(int start) {
  double avgZ = (getGyro(start) + getGyro(start - 1)) / 2.0;
  return avgZ * microsBetween(start, start - 1);
}

bool updateRevTracker() {
  double degreesSinceLastTimestamp = gyroHistory[historyCurIndex] * timeSince(-1) / 1000000;
  // % 400 (instead of 360) to account for gyro integration not being perfect, to guarantee a high+low?
  double newAngle = (revTrackingAngle + degreesSinceLastTimestamp) % 360; 
  bool hasRevolved = newAngle < revTrackingAngle;
  revTrackingAngle = newAngle;
  return hasRevolved;
}

int upIndexInLastRev() {
  double largestSmoothed = 0;
  int largestSmoothedIndex = revTrackingStartPointer;
  for (int i = revTrackingStartPointer; i != historyCurIndex - 2; i = (i + 1) % historySize) {
    double smoothed = abs(smoothedAccelHistory[i]);
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
  for (int i = upIndex; i != historyCurIndex + 1; i = (i + 1) % 60) {
    estimate += degreesTraveledForIndex(i);
  }
  return estimate;
}

int successiveLargeDiscrepancies = 0;

double updateAngleEstimate(bool revComplete) {
  bool notFirstRev = curAngleEstimate > -0.5;
  if (revComplete) {
    double estimatedFromUp = estimateAngleFromLastUp();
    bool largeDiscrepancy = notFirstRev && (abs(curAngleEstimate - estimatedFromUp) > (50 + (successiveLargeDiscrepancies * 25)));
    if (largeDiscrepancy) {
      successiveLargeDiscrepancies += 1;
    } else {
      successiveLargeDiscrepancies = 0;
    }
    if (!largeDiscrepancy) {
      curAngleEstimate = estimatedFromUp;
    }
  } else if (notFirstRev) {
    curAngleEstimate += degreesTraveledForIndex(historyCurIndex);
    curAngleEstimate = curAngleEstimate % 360;
  }
}

void sample() {
  update();
  if (isSpinning()) {
    bool revComplete = updateRevTracker();
    if (revComplete) {
      upIndex = smoothGyro();
      newUpIndex = true;
      upTimestamp = timestamps[upIndex];
      revTrackingStartPointer = historyCurIndex;
    }
    updateAngleEstimate(revComplete);
  } else {
    revTrackingAngle = 0;
    revTrackingStartPointer = historyCurIndex;
    curAngleEstimate = -1;
  }
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