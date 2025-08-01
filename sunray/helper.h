// Ardumower Sunray 
// Copyright (c) 2013-2020 by Alexander Grau, Grau GmbH
// Licensed GPLv3 for open source use
// or Grau GmbH Commercial License for commercial use (http://grauonline.de/cms2/?page_id=153)

// utilities

#ifndef HELPER_H
#define HELPER_H

#include <Arduino.h>


#define TAU TWO_PI
#define lerp(a, b, t) (a*(1.0-t) + b*t)


float scalePI(float v);
float scale180(float v);
float distancePI(float x, float w);
float distance180(float x, float w);
float distanceLineInfinite(float px, float py, float x1, float y1, float x2, float y2);
float distanceLine(float px, float py, float x1, float y1, float x2, float y2);
float fusionPI(float w, float a, float b);
double angleInterpolation(double a, double b, double t);
float scalePIangles(float setAngle, float currAngle);
float distance(float x1, float y1, float x2, float y2);
double deg2rad(double deg);
double rad2deg(double rad);
float pointsAngle(float x1, float y1, float x2, float y2);
double distanceLL(double lat1, double lon1, double lat2, double lon2);
void relativeLL(double lat1, double lon1, double lat2, double lon2, double &n, double &e);

float sign(float x);

int freeRam ();

/*
 * Returns random number in normal distribution centering on 0.
 * ~95% of numbers returned should fall between -2 and 2
 */
float gaussRandom();

/*
 * Returns member of set with a given mean and standard deviation
 * mean: mean
 * standard deviation: std_dev
 */
float gauss(float mean, float std_dev);

// calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
float gaussian(float mu, float sigma, float x);

// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float voltageDividerUges(float R1, float R2, float U2);

// ADC-value to voltage
float ADC2voltage(float ADCvalue);

// quaternion to euler angles
void toEulerianAngle(float w, float x, float y, float z, float& roll, float& pitch, float& yaw);


#define MICROS_TIME 0
#define MILLIS_TIME 1
class Timer
{
  public:
    Timer(int tt)
    {
        timeType = tt;
        update();
    }
    void update()
    {
        lastTime = now;
        now = getTime();
    }
    unsigned long deltaTime()
    {
        return now - lastTime;
    }
    double deltaTimeSeconds()
    {
        return deltaTime() / 1000000.0;
    }
    // Tau: Half-life in seconds
    float lowPass(float a, float b, double tau)
    {
        if (tau == 0.0) return b;
        double t = exp(-deltaTimeSeconds() * log(2.0) / tau);
        return lerp(b, a, t);
    }
  private:
    int timeType;
    unsigned long lastTime;
    unsigned long now;
    unsigned long getTime()
    {
        if (timeType == MICROS_TIME) return micros();
        if (timeType == MILLIS_TIME) return millis() * 1000;
        return 0;
    }
};

#endif
