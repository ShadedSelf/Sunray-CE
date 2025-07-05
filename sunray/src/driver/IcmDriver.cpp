
#include "IcmDriver.h"
#include "../../config.h"
#include "../../i2c.h"


IcmDriver::IcmDriver(){    
}

void IcmDriver::detect(){
  Wire.begin();
  Wire.setClock(400000);

  int tries = 10;
  while (tries > 0)
  {
    icm.begin(Wire, 1);
    if (icm.status != ICM_20948_Stat_Ok)
    {
      tries--;
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

bool IcmDriver::begin(){ 
  bool success = true;

  success &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
  
#if USE_MAGNOMETER
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms
#else
  success &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // 55Hz - 18 ms
#endif
  
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
  success &= (icm.setBiasCPassX(-460384) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasCPassY(-124864) == ICM_20948_Stat_Ok);
  success &= (icm.setBiasCPassZ(-2686144) == ICM_20948_Stat_Ok);

  CONSOLE.println("using imu driver: IcmDriver");
  return success;
}


void IcmDriver::run(){
}

bool IcmDriver::isDataAvail(){

    icm_20948_DMP_data_t data;
    icm.readDMPdataFromFIFO(&data);

    if ((icm.status == ICM_20948_Stat_Ok) || (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
      #if USE_MAGNOMETER
        if ((data.header & DMP_header_bitmap_Quat9) > 0)
        {
            double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
            double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
            double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      #else
        if ((data.header & DMP_header_bitmap_Quat6) > 0)
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;
      #endif

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



