// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// Ardumower battery management

#ifndef BATTERY_H
#define BATTERY_H



class Battery {
  public:
    bool docked;    // robot in docking?
	  float batteryVoltage;   // volts 
	  float systemVoltage;    // volts 
    float chargingVoltage;  // volts
    float batteryLoad;      // apms
	  float chargingCurrent;  // amps
    float temperature;
	  void begin();            
    void run();	  
    void setIsDocked(bool state);
    bool isDocked();
	  bool chargerConnected();
    bool badChargerContact();
    void enableCharging(bool flag);   	      
    bool shouldGoHome();    
    bool chargingHasCompleted();
    bool underVoltage();
    void resetIdle();
    void switchOff();
  protected:
    int chargingCompletedDelay;
    int batSwitchOffIfIdle;
    bool chargingCompleted;
    bool chargingEnabled;
    bool switchOffByOperator;    
		bool chargerConnectedState;
    bool badChargerContactState;
    unsigned long switchOffTime;
    unsigned long chargingStartTime;
	  unsigned long nextCheckTime;	  
    unsigned long nextEnableTime;		
};



#endif
