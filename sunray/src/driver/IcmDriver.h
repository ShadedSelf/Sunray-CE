
// ICM driver 

#ifndef ICM_DRIVER_H
#define ICM_DRIVER_H

#include <Arduino.h>
#include "RobotDriver.h"
#include "../icm/ICM_20948.h"

//#define ICM_20948_USE_DMP

class IcmDriver: public ImuDriver {    
  public:    
    IcmDriver();
    void detect() override;
    bool begin() override;    
    void run() override;
    bool isDataAvail() override;         
    void resetData() override;
    void getBias(
      int32_t *gyroX, int32_t *gyroY, int32_t *gyroZ,
      int32_t *accX, int32_t *accY, int32_t *accZ,
      int32_t *magX, int32_t *magY, int32_t *magZ);
    void setBias(
      int32_t gyroX, int32_t gyroY, int32_t gyroZ,
      int32_t accX, int32_t accY, int32_t accZ,
      int32_t magX, int32_t magY, int32_t magZ);
  protected:
    ICM_20948_I2C icm;
};


#endif
