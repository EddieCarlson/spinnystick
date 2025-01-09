#include "FastIMU.h"
#include <Wire.h>

const int I2C_SDA = 18;   //I2C Data pin
const int I2C_SCL = 19;   //I2c Clock pin

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;


void calibrateIMU(MPU6500 imu) {
  Serial.println("FastIMU calibration & data example");
  delay(1500);
  Serial.println("Keep IMU level.");
  delay(3000);

  imu.calibrateAccelGyro(&calib);

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

  delay(3000);
}

void initIMU(MPU6500 imu, uint32_t addr) {
  Wire.begin();
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setClock(400000); //400khz clock - max for accelerometer?

  int imuInitErr = imu.init(calib, addr);
  if (imuInitErr != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(imuInitErr);
    while (true) { ; }
  }

// TODO: indent?
#ifdef PERFORM_CALIBRATION
  calibrateIMU();
#endif

  int gyroRngErr = imu.setGyroRange(1000);
  int accRngErr = imu.setAccelRange(8);

  if (gyroRngErr != 0 || accRngErr != 0) {
    Serial.print("Error Setting range: ");
    Serial.print("gyro_range_error: ");
    Serial.print(gyroRngErr);
    Serial.print("acc_range_error: ");
    Serial.print(accRngErr);
    while (true) { ; }
  }
}
