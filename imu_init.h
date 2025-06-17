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

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

const int historySize = 150;
int historyCurIndex = 0;
int revTrackingStartIndex = 0;
double curAngleEstimate = 0;
double curAngleEstimatePadMod = 0;
double curAngleEstimateUnmodded = 0;
int upIndex = 0;
unsigned long upTimestamp = 0;
bool newUpIndex = false;
bool currentlySpinning = false;
bool firstRev = true;

const double minDegreesPerSec = 600;

double accelHistory[historySize];
double gyroHistory[historySize];
double smoothedAccelHistory[historySize];
unsigned long timestamps[historySize];

double curDegPerSec = -1;

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
  sensorLog = SD.open("sensor_log_angle_estimate_7.txt", FILE_WRITE);
  sensorLog.println("zzzzzz sensorLog opened");
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

double getSmoothedAccel(int anyIndex) { return smoothedAccelHistory[boundedHistoryIndex(anyIndex)]; }
double getCurSmoothedAccel(int offset) { return getAccel(historyCurIndex + offset); }
double getCurSmoothedAccel() { return getCurAccel(0); }

unsigned long getTimestamp(int anyIndex) { return timestamps[boundedHistoryIndex(anyIndex)]; }
unsigned long getCurTimestamp(int offset) { return getTimestamp(historyCurIndex + offset); }

unsigned long microsBetween(int startIndex, int endIndex) {
  return getTimestamp(endIndex) - getTimestamp(startIndex);
}

unsigned long timeSince(int offset) {
  return microsBetween(historyCurIndex - 1, historyCurIndex);
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
  int twoAhead = boundedHistoryIndex(historyCurIndex + 2);
  gyroHistory[twoAhead] = 0;
  accelHistory[twoAhead] = 0;
  smoothedAccelHistory[twoAhead] = 0;
  timestamps[twoAhead] = 0;
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

  accelHistory[historyCurIndex] = (double) abs(accelData.accelY);
  double gyroZX = sqrt(pow(gyroData.gyroZ, 2) + pow(gyroData.gyroX, 2));
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

int successiveLargeDiscrepancies = 0;
double lastDiscrepancy = 0;
int accumulatingDiscrepancies = 0;
bool spinningToRight = true;
double discrepancyPerDegree = 0;

double degreesTraveledForIndex(int start) {
  double avgZ = (getGyro(start - 1) + getGyro(start)) / 2.0;
  double degrees = (avgZ * microsBetween(start - 1, start) / 1000000.0);
  double discrepancyAdjustment = degrees * discrepancyPerDegree;
  degrees += discrepancyAdjustment;
  if (!spinningToRight) {
    degrees = -1 * degrees;
  }
  return degrees;
}

void resetRevTracker() {
  revTrackingStartIndex = historyCurIndex;
  firstRev = true;
  curAngleEstimate = 0;
  curAngleEstimatePadMod = 0;
}

int upIndexInLastRev() {
  int upCount = 0;
  int downCount = 0;
  double largestSmoothed = getSmoothedAccel(historyCurIndex - 2);
  int largestSmoothedIndex = boundedHistoryIndex(historyCurIndex - 2);
  for (int i = boundedHistoryIndex(historyCurIndex - 3); i != revTrackingStartIndex - 1; i = boundedHistoryIndex(i - 1)) {
    double smoothed = smoothedAccelHistory[i];
    if (smoothed > largestSmoothed) {
      upCount += 1;
      largestSmoothed = smoothed;
      largestSmoothedIndex = i;
    } else if (upCount > 3) { // we want to go up and then down (to find a local maximum)
      downCount += 1;
    }
    if (downCount > 3) {
      break;
    }
  }

  return largestSmoothedIndex;
}

double estimateAngleFromLastUp() {
  // obob?
  double estimate = 0;
  for (int i = upIndex; i != boundedHistoryIndex(historyCurIndex + 1); i = boundedHistoryIndex(i + 1)) {
    estimate += degreesTraveledForIndex(i);
  }
  return estimate;
}

int previouslyRight = 0;
double compensationFactor = 3.5;
double smoothedDiff = 0;

double updateAngleEstimate() {
  double degTraveled = degreesTraveledForIndex(historyCurIndex);
  curAngleEstimateUnmodded += degTraveled;
  curAngleEstimate += degTraveled;
  curAngleEstimate = fmod(curAngleEstimate + 360, 360);
  double oldEstimate = curAngleEstimatePadMod;
  curAngleEstimatePadMod += degTraveled;
  curAngleEstimatePadMod = fmod(curAngleEstimatePadMod + 380, 380);
  // include padding so we don't end just before the peak
  bool revComplete = curAngleEstimatePadMod < oldEstimate;
  if (revComplete) {
    double estimatedFromUp = fmod(estimateAngleFromLastUp() + 360, 360);
    bool curAngleLeftOfUp = curAngleEstimate > 180;
    bool estimatedFromUpLeftOfUp = estimatedFromUp > 180;
    double diff = 0; // positive value means FromUp calculation is "ahead" of gyro int in dir of rotation
    if (curAngleLeftOfUp && estimatedFromUpLeftOfUp) {
      diff = estimatedFromUp - curAngleEstimate;
    } else if (curAngleLeftOfUp && !estimatedFromUpLeftOfUp) {
      diff = (360 - curAngleEstimate) + estimatedFromUp;
    } else if (!curAngleLeftOfUp && estimatedFromUpLeftOfUp) {
      diff = -1.0 * ((360 - estimatedFromUp) + curAngleEstimate);
    } else {
      diff = estimatedFromUp - curAngleEstimate;
    }
    if (abs(diff) > lastDiscrepancy) {
      successiveLargeDiscrepancies += 1;
    } else {
      successiveLargeDiscrepancies = 0;
    }
    if (successiveLargeDiscrepancies > 2) {
      spinningToRight = !spinningToRight;
    }


    diff = min(max(diff, -60.0), 60.0);
    if (firstRev) {
      smoothedDiff = diff;
    } else {
      smoothedDiff = (smoothedDiff * 0.1) + (smoothedDiff * 0.9);
    }
    double absDiff = abs(smoothedDiff);
    if (absDiff < 20 && compensationFactor < 7.5) {
      compensationFactor += max(1.5 / absDiff, 0.15);
      // compensationFactor = compensationFactor * (((30 - absDiff) / 30) * 1.08);
    } else if (absDiff > 36 && compensationFactor > 3) {
      compensationFactor -= max((absDiff - 5) / (25 * 10), 0.22);
      // compensationFactor = compensationFactor / (max((absDiff - 10) / 48, 1) * 1.08);
    }
    compensationFactor = max(min(compensationFactor, 7.6), 2.9);

    // if (previouslyRight == goingRight && abs(diff) > abs(lastDiscrepancy)) {
    //   accumulatingDiscrepancies += 1;
    // } else {
    //   accumulatingDiscrepancies = 0;
    // }
    // previouslyRight = goingRight;
    // lastDiscrepancy = diff;
    // if (accumulatingDiscrepancies > 2) {
    //   spinningToRight = !spinningToRight;
    // }
    // bool largeDiscrepancy = false;
    // bool largeDiscrepancy = !firstRev &&
    //   (positiveDiscrepancy > (36 + (successiveLargeDiscrepancies * 36)));
    // bool largeDiscrepancy = !firstRev &&
    //   (abs(curAngleEstimate - estimatedFromUp) > (42 + (successiveLargeDiscrepancies * 36)));
    // if (largeDiscrepancy) {
    //   successiveLargeDiscrepancies += 1;
    // } else {
    //   discrepancyPerDegree = min(max(discrepancy / (360.0 * 3), -0.33), 0.33);
    //   successiveLargeDiscrepancies = 0;
    //   curAngleEstimate = estimatedFromUp;
    // }
    discrepancyPerDegree = diff / (360.0 * compensationFactor);
    successiveLargeDiscrepancies = 0;
    // curAngleEstimate = estimatedFromUp;

    // curAngleEstimate = estimatedFromUp;
    curAngleEstimatePadMod = curAngleEstimate;
    firstRev = false;
  }
  return revComplete;
}

void setSpeed() {
  if (curDegPerSec < -0.5) {
    curDegPerSec = getCurGyro();
  }
  curDegPerSec = (curDegPerSec * 0.4) + (getCurGyro() * 0.6);
}

int rotationCount = 0;

void sample() {
  updateHistory();
  currentlySpinning = isSpinning();
  if (currentlySpinning) {
    setSpeed();
    bool revComplete = updateAngleEstimate();
    if (revComplete) {
      rotationCount += 1;
      upIndex = upIndexInLastRev();
      upTimestamp = timestamps[upIndex];
      // don't include the first chunk of elems after last up index for considering next upIndex
      revTrackingStartIndex = boundedHistoryIndex(upIndex);
    }
  } else {
    resetRevTracker();
    curDegPerSec = -1;
    compensationFactor = 3.5;
  }
}

// TODO: only display image when stick has had a gyroZ > 400 in the last second


void printStuff() {
  if (currentlySpinning) {
    // Serial.print(historyCurIndex);
    // Serial.print("|");
    // Serial.print(getCurTimestamp(0));
    // Serial.print("|");
    // Serial.print(timeSince(-1));
    // Serial.print("|");
    // Serial.print(curAngleEstimate);
    // Serial.print("|");
    // Serial.print(curAngleEstimateUnmodded);
    // Serial.print("|");
    // Serial.print(getCurAccel());
    // Serial.print("|");
    // Serial.println(getCurGyro());
    // Serial.print("|");
    // Serial.print(accelData.accelX);
    // Serial.print("|");
    // Serial.print(accelData.accelZ);
    // Serial.print("|");
    // Serial.print(gyroData.gyroX);
    // Serial.print("|");
    // Serial.print(gyroData.gyroY);
    // Serial.print("|");
    // Serial.print(justRevolved);
    sensorLog.print(getCurTimestamp(0));
    sensorLog.print("|");
    sensorLog.print(timeSince(-1));
    sensorLog.print("|");
    sensorLog.print(curAngleEstimate);
    sensorLog.print("|");
    sensorLog.print(curAngleEstimateUnmodded);
    sensorLog.print("|");
    sensorLog.print(getCurGyro());
    sensorLog.print("|");
    sensorLog.print(getCurAccel());
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
    sensorLog.print(gyroData.gyroZ);
    sensorLog.print("|");
    sensorLog.println(rotationCount);
  }
}