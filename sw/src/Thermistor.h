#ifndef Thermistor_h
#define Thermistor_h

#include <Arduino.h>

class Thermistor
{
  public:
    Thermistor(int thermistorPin);
    double Thermistor_Read();
    
  private:
    int thermistorPin; 
};

#endif