 // Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

#include "battery.h"
#include "config.h"
#include "helper.h"
#include "motor.h"
#include "robot.h"
#include "buzzer.h"
#include <Arduino.h>

#include "LineTracker.h"

// lithium akkus sollten bis zu einem bestimmten wert CC also constant current geladen werden 
// und danach mit CV constant voltage
// um die lebensdauer zu erhöhen kann man die max spannung herabsetzen
// wir verwenden im Ardumower ein 7S (7 * 4.2V)
// voll geladen ist dann bei 29.4V schluss
// wenn man z.B. die spannung von 4.2V pro zelle auf 4.1V herab setzt (85–90% charged) kann man die lebensdauer 
// verdoppeln - das gleiche gilt bei der endladung

// lithium cells
// ardumower:  Sony US18650 VTC5, 7 cells in series, nominal volage 3.6v
// alfred:   Samsung INR18650-15M, 7 cells in series, nominal voltage 3.6v 

void Battery::begin()
{  
  nextCheckTime = 0;
  nextEnableTime = 0;
  chargingVoltage = 0;
  chargingCompletedDelay = 0;
  batteryVoltage = 0;
  systemVoltage = 0.0;
  chargerConnectedState = false;
  badChargerContactState = false;      
  chargingCompleted = false;
  chargingEnabled = true;
  docked = false;
  batSwitchOffIfIdle = 300;
  switchOffByOperator = false;

  enableCharging(false);
  resetIdle();    
}


// controls charging relay
void Battery::enableCharging(bool flag){
  if (chargingEnabled == flag) return;
  DEBUG(F("enableCharging "));
  DEBUGLN(flag);
  chargingEnabled = flag;
  batteryDriver.enableCharging(flag);       	   	   	
}

bool Battery::chargerConnected(){
  return chargerConnectedState;  
}

bool Battery::isDocked(){
  return docked;
}

void Battery::setIsDocked(bool state){
  CONSOLE.print("battery.setIsDocked ");
  CONSOLE.println(state);

  docked = state;
}

bool Battery::badChargerContact(){
  return badChargerContactState;
}
 
bool Battery::chargingHasCompleted(){
  return chargingCompleted;
}
 

bool Battery::shouldGoHome(){
  if (batteryVoltage < 0.1) return false;  
  return (batteryVoltage < GO_HOME_VOLTAGE);
}

bool Battery::underVoltage(){
  if (batteryVoltage < 0.1) return false;
  return (batteryVoltage < BAT_UNDERVOLTAGE);
}

void Battery::resetIdle(){
  switchOffTime = millis() + batSwitchOffIfIdle * 1000;    
}

void Battery::switchOff(){
  CONSOLE.println("switching-off battery by operator...");
  switchOffByOperator = true;
}

Timer batteryTimer(MICROS_TIME);
Scheduler temperatureSchedule(MICROS_TIME, 1 * 60 * 1000);

float getVoltageDropFactor()
{
  float tempChange = 25.0 - battery.temperature;
  return stanleyTrackingSlowP * (1.0 + tempChange * stanleyTrackingSlowK); //0.4 - 0.0
}

void Battery::run()
{
  // give some time to establish communication to external hardware etc.
  if (millis() <= 5000) 
    return;

  // first values after starting
  if (batteryVoltage < 0.1)
  {
    temperature = batteryDriver.getBatteryTemperature();
    if (fabs(temperature) > 900.0) temperature = 25.0;

    systemVoltage = batteryDriver.getBatteryVoltage();
    batteryVoltage = systemVoltage + getVoltageDropFactor() * (motor.motorsSenseLP + 15.0);
    chargingCurrent = batteryDriver.getChargeCurrent();
    chargingVoltage = batteryDriver.getChargeVoltage();
    batteryLoad = motor.motorsSenseLP + 0.15;
  }


  // battery temperature, every minute
  if (temperatureSchedule.shouldUpdate())
    batteryDriver.myHumidity.requestTemperatureNonBlocking();
  batteryDriver.myHumidity.readTemperatureNonBlocking(temperature);

  
  batteryTimer.update();


  // battery readings
  float cVoltage  = batteryDriver.getChargeVoltage();   
  float cCurrent  = batteryDriver.getChargeCurrent();
  float voltage   = batteryDriver.getBatteryVoltage();
  float current = motor.motorLeftSense + motor.motorRightSense + motor.motorMowSense + 0.15;
  
  if (fabs(chargingVoltage - cVoltage) > 10.0)
    chargingVoltage = cVoltage;
  
  if (fabs(cCurrent) < 0.05 && fabs(chargingCurrent) < 0.05)
    cCurrent = chargingCurrent = 0.0;

  float voltageDrop = (chargerConnected())
    ? cCurrent * -0.5
    : current * getVoltageDropFactor();

  // low pass filters /expensice 150us??
  chargingCurrent = batteryTimer.lowPassF(chargingCurrent, cCurrent, 30.0);
  chargingVoltage = batteryTimer.lowPassF(chargingVoltage, cVoltage, 30.0);
  systemVoltage   = batteryTimer.lowPassF(systemVoltage,   voltage,  30.0);
  batteryVoltage  = batteryTimer.lowPassF(batteryVoltage,  voltage + voltageDrop, 30.0);
  batteryLoad     = motor.motorsSenseLP + 0.15;

  
  // charger connected
  if (!chargerConnectedState && chargingVoltage > 7.0)
    chargerConnectedState = true;

  // disconnected
  if (chargerConnectedState && chargingVoltage <= 5.0)
  {
    chargerConnectedState = false;

    // reset on undock
    nextEnableTime = millis() + 5000;
  }


  // Charge?
  if (millis() > nextEnableTime)
  {
    nextEnableTime = millis() + 5000;  	   	   	
    
    if (chargerConnected())
    {
      // charger in connected state
      if (chargingEnabled)
      {             
        if (chargingCompletedDelay > 5)  // chargingCompleted check first after 6 * 5000ms = 30sec.
          chargingCompleted = chargingCurrent <= BAT_FULL_CURRENT || batteryVoltage >= BAT_FULL_VOLTAGE; //lower batfullvoltage when schudele mowibng is allowed?
        else     
          chargingCompletedDelay++;  
         
        if (chargingHasCompleted() || temperature > DOCK_OVERHEAT_TEMP - 1.5)
        {
          // stop charging
          enableCharging(false);
          
          // next check
          if (chargingHasCompleted())
            nextEnableTime = millis() + 1000 * BAT_FULL_CHARGE_RECONNECT;
          else
            nextEnableTime = millis() + 1000 * 60 * 10;
        } 
      }
      else if (batteryVoltage < BAT_FULL_VOLTAGE && temperature < DOCK_OVERHEAT_TEMP - 3.0)
      {
        enableCharging(true);
        chargingStartTime = millis();       
      }    
    }
    else
    { 
      // reset to avoid direct undocking after docking   
      chargingCompleted       = false; 
      chargingCompletedDelay  = 0;
    }
  }
	

  if (millis() >= nextCheckTime)
  {    
    nextCheckTime = millis() + 5000;  	

    // Keep power?
    if (underVoltage())
    {
      DEBUG(F("SWITCHING OFF (undervoltage) batV="));
      DEBUG(batteryVoltage);
      DEBUG("<");
      DEBUGLN(BAT_UNDERVOLTAGE);

      buzzer.sound(SND_OVERCURRENT, true);

      if (BAT_SWITCH_OFF_UNDERVOLTAGE)
        batteryDriver.keepPowerOn(false);     
    }
    else if (/*millis() >= switchOffTime || */switchOffByOperator)
    {
      DEBUGLN(F("SWITCHING OFF (idle timeout)"));  

      buzzer.sound(SND_OVERCURRENT, true);

      if (BAT_SWITCH_OFF_IDLE || switchOffByOperator)
        batteryDriver.keepPowerOn(false);
    }
    else 
      batteryDriver.keepPowerOn(true);
  }       
}
