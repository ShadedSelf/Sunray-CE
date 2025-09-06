
#include "IcmDriver.h"
#include "../../config.h"
#include "../../i2c.h"


IcmDriver::IcmDriver(){    
}

void IcmDriver::detect()
{
  Wire.begin();
  Wire.setClock(400000);

  int tries = 10;
  while (tries > 0)
  {
    icm.begin(Wire, 1);
    if (icm.status != ICM_20948_Stat_Ok)
    {
      tries--;
      watchdogReset();
      delay(500);
    }
    else
    {
      imuFound = true;
      CONSOLE.println(" ");
      CONSOLE.println("ICM 20948 found");
      return;
    }
  }
  imuFound = false;
  CONSOLE.println(F("ICM 20948 not found"));        
}

bool IcmDriver::begin()
{ 
  bool success = true;

  watchdogEnable(2000);
  success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
  watchdogEnable(WATCHDOG_TIMER);
  watchdogReset();
  
#if USE_MAGNOMETER
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms
#endif
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms

  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms

  //success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms
  
  success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
  success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
  success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

  // sensor biases
  success &= (icm.setBiasGyroX(24192) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasGyroY(113088) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasGyroZ(-171744) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasAccelX(0) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasAccelY(0) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasAccelZ(0) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasCPassX(-274656) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasCPassY(640480) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasCPassZ(-4016672) == ICM_20948_Stat_Ok);

  CONSOLE.println("using imu driver: IcmDriver");
  return success;
}


void IcmDriver::run(){
}

void IcmDriver::getBias(
  int32_t *gyroX, int32_t *gyroY, int32_t *gyroZ,
  int32_t *accX, int32_t *accY, int32_t *accZ,
  int32_t *magX, int32_t *magY, int32_t *magZ)
{
  icm.getBiasGyroX(gyroX);
  icm.getBiasGyroY(gyroY);
  icm.getBiasGyroZ(gyroZ);
  icm.getBiasAccelX(accX);
  icm.getBiasAccelY(accY);
  icm.getBiasAccelZ(accZ);
  icm.getBiasCPassX(magX);
  icm.getBiasCPassY(magY);
  icm.getBiasCPassZ(magZ);
}

void IcmDriver::setBias(
  int32_t gyroX, int32_t gyroY, int32_t gyroZ,
  int32_t accX, int32_t accY, int32_t accZ,
  int32_t magX, int32_t magY, int32_t magZ)
{
  icm.setBiasGyroX(gyroX);
  icm.setBiasGyroY(gyroY);
  icm.setBiasGyroZ(gyroZ);
  icm.setBiasAccelX(accX);
  icm.setBiasAccelY(accY);
  icm.setBiasAccelZ(accZ);
  icm.setBiasCPassX(magX);
  icm.setBiasCPassY(magY);
  icm.setBiasCPassZ(magZ);
}

bool IcmDriver::isDataAvail()
{
  icm_20948_DMP_data_t data;
  icm.readDMPdataFromFIFO(&data);

  icm.agmt.gyr.axes.y;

  if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    if ((data.header & DMP_header_bitmap_Gyro) > 0)
    {
      yawSpeed = data.Raw_Gyro.Data.Z;// / 16.4 / 2000.0;// / 32768.0 / 2000.0;
      //DEBUGLN(data.Raw_Gyro.Data.BiasZ / 350.0);
      //DEBUGLN(yawSpeed);
    }

    //if ((data.header & DMP_header_bitmap_Gyro_Calibr) > 0)
    //{
     // yawSpeed = data.Gyro_Calibr.Data.Z / 32768.0;// / 2000.0;
      //DEBUGLN(yawSpeed);
    //}

    if (USE_MAGNOMETER && (data.header & DMP_header_bitmap_Quat9) > 0)
    {
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;

      double q0 = sqrt(1.0 - min((q1 * q1) + (q2 * q2) + (q3 * q3), 1.0));
      double q2sqr = q2 * q2;

      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      magYaw = atan2(t3, t4);
    }

    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

      double q0 = sqrt(1.0 - min((q1 * q1) + (q2 * q2) + (q3 * q3), 1.0));
      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      roll = atan2(t0, t1);

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2);

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      yaw = atan2(t3, t4);

      //orientation = quat_t(q0, q1, q2, q3).norm();

      return true;
    }
  }
  return false;
}       
    
void IcmDriver::resetData(){
    icm.resetFIFO();
}



