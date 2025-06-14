#include "timetable.h"
#include "config.h"
#include "robot.h"
#include "src/op/op.h"


TimeTable::TimeTable()
{   
    timetable.enable = false; 

    timetable.hours[0] = 0;
    timetable.hours[1] = 0;
    timetable.hours[2] = 0;
    timetable.hours[3] = 0;
    timetable.hours[4] = 0;
    timetable.hours[5] = 0;
    timetable.hours[6] = 0;
    timetable.hours[7] = 0;
    timetable.hours[8] = 0;
    timetable.hours[9] = 127; // if mowing allowed, mask is set for that day
    timetable.hours[10] = 127;
    timetable.hours[11] = 127;
    timetable.hours[12] = 127;
    timetable.hours[13] = 127;
    timetable.hours[14] = 127;
    timetable.hours[15] = 127;
    timetable.hours[16] = 127;
    timetable.hours[17] = 127;
    timetable.hours[18] = 127;
    timetable.hours[19] = 127;
    timetable.hours[20] = 0;
    timetable.hours[21] = 0;
    timetable.hours[22] = 0;
    timetable.hours[23] = 0;    
}

// set current UTC time
void TimeTable::setCurrentTime(int hour, int min, int dayOfWeek){
    currentTime.hour = hour;
    currentTime.min = min;
    currentTime.dayOfWeek = dayOfWeek;
    CONSOLE.print("GPS time (UTC): ");
    dumpWeekTime(currentTime);
}    

void TimeTable::dumpWeekTime(weektime_t time){
    CONSOLE.print("dayOfWeek=");
    String s;
    switch (time.dayOfWeek){
        case 0: s = "mon"; break;
        case 1: s = "tue"; break;
        case 2: s = "wed"; break;
        case 3: s = "thu"; break;
        case 4: s = "fri"; break;
        case 5: s = "sat"; break;
        case 6: s = "sun"; break;         
    }    
    CONSOLE.print(s);
    CONSOLE.print("  hour=");
    CONSOLE.println(time.hour);
}


void TimeTable::dump(){
    CONSOLE.print("timetable (UTC times)    ");                
    for (int hour=0; hour < 24; hour++){
        String s;
        if (hour < 10) s += "0";
        s += hour;
        CONSOLE.print(s);
        CONSOLE.print(" ");
    }
    CONSOLE.println();
    for (int day=0; day < 7; day++){        
        CONSOLE.print("timetable (UTC times) ");
        String s;
        switch (day){
            case 0: s = "mon"; break;
            case 1: s = "tue"; break;
            case 2: s = "wed"; break;
            case 3: s = "thu"; break;
            case 4: s = "fri"; break;
            case 5: s = "sat"; break;
            case 6: s = "sun"; break;         
        }
        CONSOLE.print(s);
        int mask = (1 << day);
        for (int hour=0; hour < 24; hour++){
            String s = "   ";
            if (timetable.hours[hour] & mask) s = " * ";
            CONSOLE.print(s);
        }
        CONSOLE.println();
    }
    CONSOLE.println("* means mowing allowed");
    CONSOLE.print("current GPS UTC weektime: ");
    dumpWeekTime(currentTime);
    CONSOLE.print("timetable enabled: ");
    CONSOLE.println(timetable.enable);
    CONSOLE.print("mowing allowed: ");    
    CONSOLE.println(mowingAllowed());
}


void TimeTable::clear(){    
    for (int i=0; i  < 24; i++)
        timetable.hours[i] = 0;
}    

int TimeTable::crc(){
    int crc = 0;
    for (int i=0; i  < 24; i++)
        crc += i * timetable.hours[i];
    crc += ((byte)timetable.enable);
    return crc;
}

// set day mask for hour 
bool TimeTable::setDayMask(int hour, daymask_t mask){
    if (hour < 0 || hour > 23) return false;
    timetable.hours[hour] = mask;
    return true;
}

void TimeTable::setEnabled(bool flag){
    timetable.enable = flag;
}
    

bool TimeTable::mowingAllowed(weektime_t time){
    if (!timetable.enable) return true; // timetable not enabled => mowing allowed
    int hour = time.hour; 
    if (hour < 0 || hour > 23) return false;    
    int mask = (1 << time.dayOfWeek);

    bool allowed = ( (timetable.hours[hour] & mask) != 0); // if mowing allowed, mask is set for that day
    return allowed;
}

bool TimeTable::mowingAllowed(){
    return mowingAllowed(currentTime);
}


// called from charge operation
bool TimeTable::shouldAutostartNow(){
 return isEnabled()
  && mowingAllowed()
  && battery.isDocked()
  && battery.chargingHasCompleted()
  && millis() > dockOp.dockReasonRainAutoStartTime;
}


// called from mow operation
bool TimeTable::shouldAutostopNow(){
    return isEnabled() && !mowingAllowed();
}

bool TimeTable::isEnabled(){
    return timetable.enable;
}

// called every 30s in robot
void TimeTable::run()
{    

}
