#include "FastIMU.h"
#include <Wire.h>

#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment out this line to skip calibration at start
MPU6500 IMU;               //Change to the name of any supported IMU!
// Other supported IMUS: MPU9255 MPU9250 MPU6886 MPU6050 ICM20689 ICM20690 BMI055 BMX055 BMI160 LSM6DS3 LSM6DSL

const int I2C_SDA = 18;   //I2C Data pin
const int I2C_SCL = 19;   //I2c Clock pin

calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;

void calibrateIMU() {
  Serial.println("FastIMU calibration & data example");
  delay(1500);
  Serial.println("Keep IMU level.");
  delay(3000);

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

  delay(3000);
}

void initIMU() {
  Wire.begin();
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.setClock(400000); //400khz clock - max for accelerometer?

  int imuInitErr = IMU.init(calib, IMU_ADDRESS);
  if (imuInitErr != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(imuInitErr);
    while (true) { ; }
  }

// TODO: indent?
#ifdef PERFORM_CALIBRATION
  calibrateIMU();
#endif

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
