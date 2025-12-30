#include <Wire.h>
#include <SFE_LSM9DS0.h>
#include <math.h>

///////////////////////
// Example I2C Setup //
///////////////////////
// Comment out this section if you're using SPI
// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// Create an instance of the LSM9DS0 library called `dof` the
// parameters for this constructor are:
// [SPI or I2C Mode declaration],[gyro I2C address],[xm I2C add.]
LSM9DS0 imu(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

float ax, ay, az, gx, gy, gz, mx, my, mz;
float compensated_mag_yaw;
float accelPitch, accelRoll;

void setup()
{
  Serial.begin(115200); // Start serial at 115200 bps

  uint16_t status = imu.begin();

  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();
}

void loop()
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();

  ax = (float) imu.calcAccel(imu.ax); 
  ay = (float) imu.calcAccel(imu.ay);
  az = (float) imu.calcAccel(imu.az);
  gx = (float) imu.calcGyro(imu.gx); 
  gy = (float) imu.calcGyro(imu.gy);
  gz = (float) imu.calcGyro(imu.gz);
  mx = (float) imu.calcMag(imu.mx); 
  my = (float) imu.calcMag(imu.my);
  mz = (float) imu.calcMag(imu.mz);

  accelPitch = atan2 (ay ,( sqrt ((ax * ax) + (az * az))));
  accelRoll = atan2(-ax ,( sqrt((ay * ay) + (az * az))));

  float Yh = (my * cos(accelRoll)) - (mz * sin(accelRoll));
  float Xh = (mx * cos(accelPitch))+(my * sin(accelRoll)*sin(accelPitch))+(mz * cos(accelRoll) * sin(accelPitch));

  compensated_mag_yaw = atan2(Yh, Xh);
  compensated_mag_yaw = (compensated_mag_yaw * 180.0) / M_PI;
  Serial.println(compensated_mag_yaw);

  // Serial.print(mx); Serial.print(',');
  // Serial.print(my); Serial.print(',');
  // Serial.print(mz); Serial.print(',');
  // Serial.print(gx); Serial.print(',');
  // Serial.print(gy); Serial.print(',');
  // Serial.print(gz); Serial.print(',');
  // Serial.print(ax); Serial.print(',');
  // Serial.print(ay); Serial.print(',');
  // Serial.println(az);

}


