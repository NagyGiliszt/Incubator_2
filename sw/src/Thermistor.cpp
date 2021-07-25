#include "Thermistor.h"

// ellenállás 25 ° C-on
#define THERMISTORNOMINAL 10000      
// Hőm. névleges ellenálláshoz (szinte mindig 25 ° C)
#define TEMPERATURENOMINAL 25   
// Mintavételezés száma
// Ha több, lassabb, de pontosabb 
#define NUMSAMPLES 100
// A termisztor béta-együtthatója (általában 3000-4000)
#define BCOEFFICIENT 3950
// az „egyéb” (soros pozitív felé) ellenállás értéke
#define SERIESRESISTOR 10000  

int samples[NUMSAMPLES];

Thermistor::Thermistor(int ThermistorPin)
{
  pinMode(ThermistorPin, INPUT);
  thermistorPin = ThermistorPin;
}

double Thermistor::Thermistor_Read()
{
  uint8_t i;
  double average;

  // vegyen N mintát egymás után, kissé késleltetve
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(thermistorPin);
   delay(2);
  }
  
  // az összes mintát átlagolja
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

  // konvertálja az értéket ellenállásra
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;

  // https://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
  
  double Temp;
  Temp = average / THERMISTORNOMINAL;            // (R/Ro)
  Temp = log(Temp);                              // ln(R/Ro)
  Temp /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)
  Temp += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  Temp = 1.0 / Temp;                            // Invert
  Temp -= 273.15;                               // Kelvin to Celsius

  return Temp;

}