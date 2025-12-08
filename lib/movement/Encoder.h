#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
private:
    volatile long tick = 0;                
    unsigned long timestamp = 0; 
    unsigned long tickInterval = 0; 
    int resolution = 1;

public:
    Encoder();                        
    Encoder(int resolution);          
    void addTick();
    void subtractTick();                 
    long getTicks();                   
    void changeResolution(int newResolution); 
    void setTimestamp(unsigned long time);   
    unsigned long getTimestamp();    
    void setTickInterval(unsigned long time); 
    unsigned long getTickInterval();  
    void reset();
    float getRevolutions(); 
    float getRPM();
};

#endif

