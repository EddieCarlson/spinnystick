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
  sensorLog = SD.open("sensor_log.txt", FILE_WRITE);
  Wire2.begin();
  Wire2.setSDA(I2C_SDA);
  Wire2.setSCL(I2C_SCL);
  Wire2.setClock(400000); //400khz clock - max for accelerometer?

  int imuInitErr = IMU.init(calib, IMU_ADDRESS);
  if (imuInitErr != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(imuInitErr);
    while (true) { ; }
  }

  int gyroRngErr = IMU.setGyroRange(1000);
  int accRngErr = IMU.setAccelRange(8);

  if (gyroRngErr != 0 || accRngErr != 0) {
    Serial.print("Error Setting range: ");
    Serial.print("gyro_range_error: ");
    Serial.print(gyroRngErr);
    Serial.print("acc_range_error: ");
    Serial.print(accRngErr);
    while (true) { ; }
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
  sensorLog.flush();
  sensorLog.close();
  delay(1000);
}

void printStuff() {
  loopCount++;
  if (loopCount % 11 == 0) {
    sensorLog.println("-----------------------");
  }
  unsigned long start = micros();
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  unsigned long microdiff = micros() - start;
  sensorLog.print("micros for updates: " + microdiff + '\n');
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