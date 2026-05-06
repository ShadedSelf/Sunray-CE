
#include "IcmDriver.h"
#include "../../config.h"
#include "../../i2c.h"
#include "../../helper.h"


IcmDriver::IcmDriver(){    
}

void IcmDriver::detect()
{
  Wire.begin();
  Wire.setClock(400000);
  //Wire.setClock(1000000);

  int tries = 10;
  while (tries > 0)
  {
    icm.begin(Wire, 1);
    if (icm.status != ICM_20948_Stat_Ok)
    {
      tries--;
      watchdogReset();
      delay(500);
      watchdogReset();
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
  newData = false;

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
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms

  //success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON) == ICM_20948_Stat_Ok);
  //success &= (icm.setDMPODRrate(DMP_ODR_Reg_ALS, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms

  //success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (icm.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms
  
  success &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (icm.enableDMP() == ICM_20948_Stat_Ok);
  success &= (icm.resetDMP() == ICM_20948_Stat_Ok);
  success &= (icm.resetFIFO() == ICM_20948_Stat_Ok);

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
  newData = false;

  icm_20948_DMP_data_t data;
  icm.readDMPdataFromFIFO(&data);

  if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    if ((data.header & DMP_header_bitmap_Gyro) > 0)
    {
      yawSpeed = (data.Raw_Gyro.Data.Z - (data.Raw_Gyro.Data.BiasZ >> 5)) * 0.001;
    }

    /*if ((data.header & DMP_header2_bitmap_Activity_Recog) > 0)
    {
      DEBUGLN(data.Activity_Recognition.Data.State_End.Still);
    }*/

    /*if ((data.header & DMP_header_bitmap_Accel) > 0)
    {
      acceleration = vec3_t(
        (double)data.Raw_Accel.Data.X / 8192.0,
        (double)data.Raw_Accel.Data.Y / 8192.0,
        (double)data.Raw_Accel.Data.Z / 8192.0);
    }*/
    
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
      newData = true;

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

      /*int32_t gyroX, gyroY, gyroZ;
      icm.getBiasGyroX(&gyroX);
      icm.getBiasGyroY(&gyroY);
      icm.getBiasGyroZ(&gyroZ);
      icm.getAGMT();

      DEBUG(icm.temp());
      DEBUG(" ");
      DEBUG(gyroX);
      DEBUG(" ");
      DEBUG(gyroY);
      DEBUG(" ");
      DEBUGLN(gyroZ);*/

      return true;
    }
  }
  return false;
}       
    
void IcmDriver::resetData(){
    icm.resetFIFO();
}



