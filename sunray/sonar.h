// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// HC-SR04 ultrasonic sensor driver (2cm - 400cm)
// for 3 sensors, optimized for speed: based on hardware interrupts (no polling)
// up to 100 Hz measurements tested

#ifndef SONAR_H
#define SONAR_H



class Sonar {
    public:      
		bool enabled;
	    float triggerLeftBelow;
        float triggerCenterBelow;
        float triggerRightBelow;
	    void begin();            
        void run();
	    bool obstacle();	    
		bool nearObstacle();
		float distanceLeft; // cm
		float distanceRight;
		float distanceCenter;  		
		bool verboseOutput; 
    protected:                 
		float convertCm(unsigned int echoTime);
		unsigned long nearObstacleTimeout;
};



#endif

