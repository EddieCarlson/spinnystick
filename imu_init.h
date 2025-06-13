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

File sensorLog;


#define orientationHistorySize 5
#define smoothingWindow 5

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

const int historySize = 60;
int historyCurIndex = 0;
int revTrackingStartIndex = 0;
double curAngleEstimate = 0;
double curAngleEstimatePadMod = 0;
double curAngleEstimateUnmodded = 0;
int upIndex = 0;
int upTimestamp = 0;
bool newUpIndex = false;
bool currentlySpinning = false;
bool firstRev = true;
bool justRevolved = false;

const double minDegreesPerSec = 600;

double accelHistory[historySize];
double gyroHistory[historySize];
double smoothedAccelHistory[historySize];
unsigned long timestamps[historySize];


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

void clearHistories() {
  for (int i = 0; i < historySize; i++) {
    gyroHistory[i] = (double) 0;
    accelHistory[i] = (double) 0;
    smoothedAccelHistory[i] = (double) 0;
    timestamps[i] = (unsigned long) 0;
  }
}

void initIMU() {
  sensorLog = SD.open("sensor_log_angle_estimate_2.txt", FILE_WRITE);
  sensorLog.println("sensorLog opened");
  Wire2.begin();
  Wire2.setSDA(I2C_SDA);
  Wire2.setSCL(I2C_SCL);
  Wire2.setClock(400000); //400khz clock - max for accelerometer?

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  clearHistories();
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
double getCurGyro() { return getCurGyro(0); }

double getAccel(int anyIndex) { return accelHistory[boundedHistoryIndex(anyIndex)]; }
double getCurAccel(int offset) { return getAccel(historyCurIndex + offset); }
double getCurAccel() { return getCurAccel(0); }

unsigned long getTimestamp(int anyIndex) { return timestamps[boundedHistoryIndex(anyIndex)]; }
unsigned long getCurTimestamp(int offset) { return getTimestamp(historyCurIndex + offset); }

unsigned long microsBetween(int startIndex, int endIndex) {
  return timestamps[boundedHistoryIndex(endIndex)] - timestamps[boundedHistoryIndex(startIndex)];
}

unsigned long timeSince(int offset) {
  return timestamps[historyCurIndex] - timestamps[getCurTimestamp(offset)];
}

// smoothedAccelHistory[i] will equal: weights [0.08, 0.12, 0.6, 0.12, 0.08] * raw accel values centered on curHistoryIndex
// smoothedAccelHistory[i] will only be complete once the curHistoryIndex = i + 2
void setSmoothedAccel() {
  double smoothedInit = (accelData.accelY * 0.6) + (getAccel(historyCurIndex - 1) * 0.12) + (getAccel(historyCurIndex - 2) * 0.08);
  smoothedAccelHistory[historyCurIndex] = smoothedInit;
  smoothedAccelHistory[boundedHistoryIndex(historyCurIndex - 1)] += accelData.accelY * 0.12;
  smoothedAccelHistory[boundedHistoryIndex(historyCurIndex - 2)] += accelData.accelY * 0.08;
}

void clearUpcoming() {
  int threeAhead = boundedHistoryIndex(historyCurIndex + 3);
  gyroHistory[threeAhead] = 0;
  accelHistory[threeAhead] = 0;
  smoothedAccelHistory[threeAhead] = 0;
  timestamps[threeAhead] = 0;
}

unsigned long updateIMU() {
  unsigned long startMicros = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  return startMicros;
}

void updateHistory() {
  historyCurIndex = boundedHistoryIndex(historyCurIndex + 1);

  unsigned long updateMicros = updateIMU();
  timestamps[historyCurIndex] = updateMicros;

  // Serial.println(accelData.accelY);
  accelHistory[historyCurIndex] = (double) abs(accelData.accelY);
  double gyroZX = sqrt(pow(gyroData.gyroZ, 2) + pow(gyroData.gyroX, 2));
  // Serial.println(gyroZX);
  gyroHistory[historyCurIndex] = gyroZX;

  setSmoothedAccel();
  clearUpcoming();
}

bool isSpinning() {
  if (currentlySpinning) {
    return getCurGyro() > minDegreesPerSec;
  } else {
    int start = -1 * ((int) floor(historySize * 0.7));
    for (int i = start; i <= 0; i++) {
      if (getCurGyro(i) < minDegreesPerSec) {
        return false;
      }
    }
    return true;
  }
}

double degreesTraveledForIndex(int start) {
  double avgZ = (getGyro(start) + getGyro(start - 1)) / 2.0;
  return avgZ * microsBetween(start, start - 1);
}

void resetRevTracker() {
  revTrackingStartIndex = historyCurIndex;
  firstRev = true;
  curAngleEstimate = 0;
  curAngleEstimatePadMod = 0;
}

int upIndexInLastRev() {
  double largestSmoothed = 0;
  int largestSmoothedIndex = revTrackingStartIndex;
  for (int i = revTrackingStartIndex; i != boundedHistoryIndex(historyCurIndex - 2); i = (i + 1) % historySize) {
    double smoothed = smoothedAccelHistory[i];
    if (smoothed > largestSmoothed) {
      largestSmoothed = smoothed;
      largestSmoothedIndex = i;
    }
  }

  return largestSmoothedIndex;
}

// TODO: try to keep it so historyCurIndex is "ahead" of upIndex by less than 1/4 the historySize, to minimize gyroZX-summation errors
double estimateAngleFromLastUp() {
  // obob?
  double estimate = 0;
  for (int i = upIndex; i != boundedHistoryIndex(historyCurIndex + 1); i = (i + 1) % 60) {
    estimate += degreesTraveledForIndex(i);
  }
  return estimate;
}

int successiveLargeDiscrepancies = 0;

double updateAngleEstimate() {
  double degTraveled = degreesTraveledForIndex(historyCurIndex);
  curAngleEstimateUnmodded += degTraveled;
  curAngleEstimate += degTraveled;
  curAngleEstimate = fmod(curAngleEstimate, 360);
  double oldEstimate = curAngleEstimatePadMod;
  curAngleEstimatePadMod += degTraveled;
  curAngleEstimatePadMod = fmod(curAngleEstimatePadMod, 400);
  // include padding so we don't end just before the peak
  bool revComplete = curAngleEstimatePadMod < oldEstimate;
  if (revComplete) {
    double estimatedFromUp = estimateAngleFromLastUp();
    bool largeDiscrepancy = !firstRev &&
      (abs(curAngleEstimate - estimatedFromUp) > (36 + (successiveLargeDiscrepancies * 36)));
    if (largeDiscrepancy) {
      successiveLargeDiscrepancies += 1;
    } else {
      successiveLargeDiscrepancies = 0;
      curAngleEstimate = estimatedFromUp;
    }
    curAngleEstimatePadMod = curAngleEstimate;
    firstRev = false;
  }
  return revComplete;
}

void sample() {
  updateHistory();
  currentlySpinning = isSpinning();
  if (currentlySpinning) {
    bool revComplete = updateAngleEstimate();
    if (revComplete) {
      justRevolved = true;
      upIndex = upIndexInLastRev();
      upTimestamp = timestamps[upIndex];
      // don't bother including the first chunk of elems after last up index for considering next upIndex
      revTrackingStartIndex = boundedHistoryIndex(upIndex + floor(historySize * 0.15));
    }
  } else {
    justRevolved = false;
    resetRevTracker();
  }
}

// TODO: only display image when stick has had a gyroZ > 400 in the last second


void printStuff() {
  if (true) {
    // Serial.print(historyCurIndex);
    // Serial.print("hi");
    // Serial.print((long) (getCurTimestamp(0) / 1000));
    // Serial.print("|");
    // Serial.print((long) (timeSince(-1) / 1000));
    // Serial.print("|");
    // Serial.print(curAngleEstimate);
    // Serial.print("|");
    // Serial.print(curAngleEstimateUnmodded);
    // Serial.print("|");
    // Serial.print(accelData.accelX);
    // Serial.print("|");
    Serial.print(getCurAccel());
    Serial.print("|");
    // Serial.print(accelData.accelZ);
    // Serial.print("|");
    // Serial.print(gyroData.gyroX);
    // Serial.print("|");
    // Serial.print(gyroData.gyroY);
    // Serial.print("|");
    Serial.println(getCurGyro());
    // Serial.print("|");
    // Serial.print(justRevolved);
    // sensorLog.print((long) (getCurTimestamp(0) / 1000));
    // sensorLog.print("|");
    // sensorLog.print((long) (timeSince(-1) / 1000));
    // sensorLog.print("|");
    // sensorLog.print(curAngleEstimate);
    // sensorLog.print("|");
    // sensorLog.print(curAngleEstimateUnmodded);
    // sensorLog.print("|");
    // sensorLog.print(accelData.accelX);
    // sensorLog.print("|");
    // sensorLog.print(accelData.accelY);
    // sensorLog.print("|");
    // sensorLog.print(accelData.accelZ);
    // sensorLog.print("|");
    // sensorLog.print(gyroData.gyroX);
    // sensorLog.print("|");
    // sensorLog.print(gyroData.gyroY);
    // sensorLog.print("|");
    // sensorLog.println(gyroData.gyroZ);
    // sensorLog.print("|");
    // sensorLog.print(justRevolved);
  } else {
    // sensorLog.flush();
  }
}