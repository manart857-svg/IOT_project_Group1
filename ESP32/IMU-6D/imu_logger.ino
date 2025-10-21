#include <Wire.h>
#include <MPU9250_WE.h>
#include "Filters.h"
#include "MadgwickAHRS.h"

#define MPU9250_ADDR 0x68
MPU9250_WE imu(MPU9250_ADDR);
const uint16_t SERIAL_BAUD = 115200;
const float TARGET_HZ = 100.0f;
const uint32_t DT_MS = (uint32_t)(1000.0f / TARGET_HZ);

FilterBank accXFilt, accYFilt, accZFilt;
FilterBank gyrXFilt, gyrYFilt, gyrZFilt;
MadgwickAHRS ahrs;

void printHeader() {
  Serial.println("time_ms,ax,ay,az,gx,gy,gz,ax_ema,ay_ema,az_ema,gx_ema,gy_ema,gz_ema,ax_median,ay_median,az_median,gx_median,gy_median,gz_median,ax_lp,ay_lp,az_lp,gx_lp,gy_lp,gz_lp,ax_outlier,ay_outlier,az_outlier,gx_outlier,gy_outlier,gz_outlier,q0,q1,q2,q3");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  imu.init();
  delay(800);
  imu.autoOffsets();
  imu.setAccRange(MPU9250_ACC_RANGE_2G);
  imu.setGyroRange(MPU9250_GYRO_RANGE_250);
  imu.enableAccDLPF(true);
  imu.setAccDLPF(MPU9250_DLPF_6);
  imu.enableGyroDLPF(true);
  imu.setGyroDLPF(MPU9250_DLPF_6);
  imu.setSampleRateDivider(9);
  const float fs = TARGET_HZ;
  accXFilt.configure(fs); accYFilt.configure(fs); accZFilt.configure(fs);
  gyrXFilt.configure(fs); gyrYFilt.configure(fs); gyrZFilt.configure(fs);
  ahrs.setSamplePeriod(1.0f / fs);
  printHeader();
}

void loop() {
  const uint32_t t0 = millis();
  xyzFloat gVal = imu.getGValues();
  float ax = gVal.x * 9.80665f;
  float ay = gVal.y * 9.80665f;
  float az = gVal.z * 9.80665f;
  xyzFloat dps = imu.getGyrValues();
  float gx = dps.x * (PI / 180.0f);
  float gy = dps.y * (PI / 180.0f);
  float gz = dps.z * (PI / 180.0f);
  FilterOutputs fax = accXFilt.update(ax);
  FilterOutputs fay = accYFilt.update(ay);
  FilterOutputs faz = accZFilt.update(az);
  FilterOutputs fgx = gyrXFilt.update(gx);
  FilterOutputs fgy = gyrYFilt.update(gy);
  FilterOutputs fgz = gyrZFilt.update(gz);
  ahrs.update(gx, gy, gz, ax, ay, az);
  float q0, q1, q2, q3;
  ahrs.getQuaternion(q0, q1, q2, q3);
  const uint32_t now_ms = millis();
  Serial.print(now_ms); Serial.print(',');
  Serial.print(ax, 6); Serial.print(',');
  Serial.print(ay, 6); Serial.print(',');
  Serial.print(az, 6); Serial.print(',');
  Serial.print(gx, 6); Serial.print(',');
  Serial.print(gy, 6); Serial.print(',');
  Serial.print(gz, 6); Serial.print(',');
  Serial.print(fax.ema, 6); Serial.print(',');
  Serial.print(fay.ema, 6); Serial.print(',');
  Serial.print(faz.ema, 6); Serial.print(',');
  Serial.print(fgx.ema, 6); Serial.print(',');
  Serial.print(fgy.ema, 6); Serial.print(',');
  Serial.print(fgz.ema, 6); Serial.print(',');
  Serial.print(fax.median, 6); Serial.print(',');
  Serial.print(fay.median, 6); Serial.print(',');
  Serial.print(faz.median, 6); Serial.print(',');
  Serial.print(fgx.median, 6); Serial.print(',');
  Serial.print(fgy.median, 6); Serial.print(',');
  Serial.print(fgz.median, 6); Serial.print(',');
  Serial.print(fax.lp, 6); Serial.print(',');
  Serial.print(fay.lp, 6); Serial.print(',');
  Serial.print(faz.lp, 6); Serial.print(',');
  Serial.print(fgx.lp, 6); Serial.print(',');
  Serial.print(fgy.lp, 6); Serial.print(',');
  Serial.print(fgz.lp, 6); Serial.print(',');
  Serial.print(fax.outlier, 6); Serial.print(',');
  Serial.print(fay.outlier, 6); Serial.print(',');
  Serial.print(faz.outlier, 6); Serial.print(',');
  Serial.print(fgx.outlier, 6); Serial.print(',');
  Serial.print(fgy.outlier, 6); Serial.print(',');
  Serial.print(fgz.outlier, 6); Serial.print(',');
  Serial.print(q0, 6); Serial.print(',');
  Serial.print(q1, 6); Serial.print(',');
  Serial.print(q2, 6); Serial.print(',');
  Serial.println(q3, 6);
  const uint32_t elapsed = millis() - t0;
  if (elapsed < DT_MS) delay(DT_MS - elapsed);
}
