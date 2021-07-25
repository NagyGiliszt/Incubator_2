/*
  [env:uno]
  platform = atmelavr
  board = uno
  framework = arduino
  lib_deps = 
    adafruit/Adafruit Unified Sensor@^1.1.4
    adafruit/DHT sensor library@^1.4.2
    fmalpartida/LiquidCrystal@^1.5.0
    soligen2010/ClickEncoder@0.0.0-alpha+sha.9337a0c46c
    alextaujenis/RBD_Servo@^1.0.2
    paulstoffregen/TimerOne@^1.1
  build_type = release

  ;upload_command = -p m328p -c usbasp -P usb -U flash:w:firmware.hex
  [platformio]
  description = Incubator V.: 2.0.0
*/


//#define USE_HUMIDIFIER    // Páratartalom beállítása
//#define USE_EGGMOTOR      // ??? Valami lesz ???
//#define I2C_LCD           // I2C lcd
//#define USE_Triac         // AC fűtés használata!!!!

#include <Arduino.h>
#include <Wire.h>
/*
  EEPROM
*/
#include <EEPROM.h>
#define EEADDR 200      // EEPROM adat cím

/*
  Rotary encoder + sw
  Open:           0
  Close:          1
  Pressed:        2
  Held:           3
  Released:       4
  Clicked:        5
  Double Clicked: 6
*/
#include <ClickEncoder.h>
#include <TimerOne.h>

int16_t oldEncPos, encPos;
uint8_t buttonState;

#define pinA 5
#define pinB 6
#define pinSw 7 //switch
#define STEPS 4

ClickEncoder encoder(pinA, pinB, pinSw, STEPS);

/*
  Servo motor
*/
#ifdef USE_HUMIDIFIER
  #include <RBD_Servo.h>
  RBD::Servo servo1(1, 1000, 2000); //pin 5, 1ms - 2ms pulse (SG90)
#endif
#ifdef USE_EGGMOTOR
  #ifndef USE_HUMIDIFIER
    #include <RBD_Servo.h>
  #endif
  RBD::Servo servo2(0, 1000, 2000);
#endif

/*
  DHT XX
*/
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 8        // DHTXX data
#define DHTTYPE DHT22   // DHT XX

//DHT dht(DHTPIN, DHTTYPE);
DHT_Unified dht(DHTPIN, DHTTYPE);
double prevhum = 0.0;

/*
  LCD
*/
#ifdef I2C_LCD
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x38);  // Set the LCD I2C address
#else
  #include <LiquidCrystal.h>
  // lcd( rs, en, D4, D5, D6, D7)
  LiquidCrystal lcd(13, 11, 12, A3, A4, A5);
#endif

#include "chars.h"

int lcdtime = 0;

/*
  NTC 10k termisztor
*/
#include "Thermistor.h"

Thermistor t_1air (A0); // NTC 1 levegő
Thermistor t_2air (A1); // NTC 2 levegő
Thermistor t_heat (A2); // NTC 3 fűtőegység

double T1;        // Levegő hőm.1
double T2;        // Levegő hőm.2
double TH;        // Fűtőtest hőm.
double airTemp;   // Levegő hőm., T1+T2 átlaga



#ifdef USE_Triac
  #include <RBDdimmer.h>

  #define outputPin  10 
  //#define zerocross  2

  dimmerLamp dimmer(outputPin);

  int outMin = 0;    // pwm min: 0
  int outMax = 100;  // pwm max: 100
#else

  /*
    PWM heat
  */
  uint8_t heat = 10;  // Fűtés
  
  int outMin = 0;    // pwm min: 0
  int outMax = 255;  // pwm max: 255
#endif

/* Out */
int pwmOut;

/*
  PWM vent
*/
uint8_t vent = 9;   // Ventillátor

/*
  LED
*/
uint8_t LedErr = 4;      // Hiba Led
uint8_t LedRun = 3;      // Inkubálás fut

/* PID  */
double Kp = 0.7;      // Hibajel(P)
double Ki = 0.005;     // Hibajel integrál(I)
double Kd = 1.0;      // Hibajel változási sebesség, derivált(D)

double last_error = 0.0;
double totalError = 0.0;
double error = 0.0;
double pidTerm = 0.0;
double changeError = 0.0;

/*
  EEPROM adat hőmérséklet
*/
struct StoreData_H {
   double kozep;         // Célhőmérséklet
   double hyst;          // Hiszterézis
   int hum;              // páratartalom
   int pwm_vent;         // Ventillátor
   int runflag;          // Inkubálás fusson e, vagy sem
};

StoreData_H StoreData = {};

/*
  millis()
*/
unsigned long previousMillis = 0;
unsigned long Time, prevTime, lastTime;

/*
  Menü
*/
int menu = 1;
void updateMenu();
void timerIsr();

void setup() {
  Serial.begin(115200);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  encoder.setAccelerationEnabled(true);
  oldEncPos = 1;

  #ifdef USE_HUMIDIFIER
    servo1.moveToDegrees(0);
  #endif
  #ifdef USE_EGGMOTOR
    servo2.moveToDegrees(0);
  #endif

  analogReference(DEFAULT); // 5V
  
  dht.begin();
  sensor_t sensor;
  dht.humidity().getSensor(&sensor);

  #ifdef USE_Triac
    dimmer.begin(NORMAL_MODE, ON);
    dimmer.setPower(0);
    dimmer.setState(OFF);
  #else
    pinMode(heat, OUTPUT);  // PWM
    digitalWrite(heat, LOW);
  #endif

  pinMode(vent, OUTPUT);  // PWM
  digitalWrite(vent, LOW);

  pinMode(LedErr, OUTPUT);
  digitalWrite(LedErr, LOW);

  pinMode(LedRun, OUTPUT);
  digitalWrite(LedRun, LOW);

  lcd.begin(20,4);
  // ékezetes karakterek

  lcd.createChar(1, CHARS[10]); // é
  lcd.createChar(2, CHARS[16]); // ő
  lcd.createChar(3, CHARS[17]); // ű
  lcd.createChar(4, CHARS[9]);  // á

  lcd.setCursor(4,0);
  lcd.print(F("Incubator"));
  lcd.setCursor(3,1);
  lcd.print(F("SW.Ver.: 2.0.6"));
  lcd.setCursor(3,2);
  lcd.print(F("HW.Ver.: 2.0.2"));
  lcd.setCursor(3,3);
  #ifdef USE_Triac
    lcd.print(F("F"));lcd.write(3);lcd.print(F("t"));lcd.write(1);lcd.print(F("s : AC"));
  #else
    lcd.print(F("F"));lcd.write(3);lcd.print(F("t"));lcd.write(1);lcd.print(F("s : DC"));
  #endif
  delay(3000);

  EEPROM.get(EEADDR, StoreData);

  /* 
    Ha az eeprom üres?, vagy valami felülírta mással, akkor alapbeállítás, egyébként eeprom adatok
  */
  if (StoreData.runflag == 0) {}
  else if (StoreData.runflag == 1) {}
  else {
    StoreData.kozep = 34.0 ;
    StoreData.hyst = 0.2;
    StoreData.hum = 80;
    StoreData.pwm_vent = 35;
    StoreData.runflag = 0;
    EEPROM.put(EEADDR, StoreData);
  }


  /* Menü indít */
  updateMenu();
}
/* Setup */

void wait_ms(const unsigned long interval){

  previousMillis = millis();
  do {
    if (!(encoder.getButton() == 4 )){
      StoreData.runflag = !StoreData.runflag;
      EEPROM.put(EEADDR,StoreData);
      while(!(encoder.getButton() == 4 ));
    }
  } while((millis() - previousMillis < interval) || (!(encoder.getButton() == 4 ) ));  
}
/* wait_ms */

void set_kozep(){
  lcd.clear();
  lcd.home();
  lcd.print(F("C")); lcd.write(1); lcd.print(F("l h"));lcd.write(2); lcd.print(F("m"));
  lcd.setCursor(0,1);
  lcd.print(StoreData.kozep); lcd.write(178);lcd.print(F("C"));
  do{
    encPos += encoder.getValue();

    if (encPos != oldEncPos) {
      if (encPos < oldEncPos) {
        StoreData.kozep = StoreData.kozep - 0.1;

        if (StoreData.kozep < 0.0) {
          StoreData.kozep = 0.0;
        } 

        lcd.setCursor(0,1);
        lcd.print(StoreData.kozep);
        lcd.print(F(" ")); lcd.write(178);lcd.print(F("C"));
        wait_ms(200L);

      } else if (encPos > oldEncPos) {
        StoreData.kozep = StoreData.kozep + 0.1;

        if (StoreData.kozep > 50.0) {
          StoreData.kozep = 50.0;
        }

        lcd.setCursor(0,1);
        lcd.print(StoreData.kozep);
        lcd.print(F(" ")); lcd.write(178);lcd.print(F("C"));
        wait_ms(200L);
      }
      oldEncPos = encPos;
    }
  } while(!(encoder.getButton() == 5 ));
  encPos = 1;
  EEPROM.put(EEADDR,StoreData);
  updateMenu();
}
/* set_kozep */

void set_hyst(){
  lcd.clear();
  lcd.home();
  lcd.print(F("Elt")); lcd.write(1); lcd.print(F("r +-"));
  lcd.setCursor(0,1);
  lcd.print(StoreData.hyst);

  do{
    encPos += encoder.getValue();

    if (encPos != oldEncPos) {
      if (encPos < oldEncPos) {
        StoreData.hyst = StoreData.hyst - 0.1;

        if (StoreData.hyst < 0.1) {
          StoreData.hyst = 0.1;
        } 

        lcd.setCursor(0,1);
        lcd.print(StoreData.hyst);
        lcd.print(F(" ")); lcd.write(178);lcd.print(F("C"));
        wait_ms(200L);
      } else if (encPos > oldEncPos) {
        StoreData.hyst = StoreData.hyst + 0.1;

        if (StoreData.hyst > 10.0) {
          StoreData.hyst = 10.0;
        }

        lcd.setCursor(0,1);
        lcd.print(StoreData.hyst);
        lcd.print(F(" ")); lcd.write(178);lcd.print(F("C"));
        wait_ms(200L);
      }
      oldEncPos = encPos;
    }
  } while(!(encoder.getButton() == 5 ));
  encPos = 1;
  EEPROM.put(EEADDR,StoreData); 
  updateMenu();
}
/* set_hyst */

void set_vent() {
  lcd.clear();
  lcd.home();
  lcd.print(F("Vent"));
  lcd.setCursor(0,1);
  lcd.print(int(StoreData.pwm_vent/2.5));

  do{
    encPos += encoder.getValue();

    if (encPos != oldEncPos) {
      if (encPos < oldEncPos) {
        StoreData.pwm_vent =  StoreData.pwm_vent - 5;

        if (StoreData.pwm_vent <= 34) {
          StoreData.pwm_vent = 0;
        } 

        lcd.setCursor(0,1);
        lcd.print(int(StoreData.pwm_vent/2.5));
        lcd.print(F(" %"));
        wait_ms(200L);
      } else if (encPos > oldEncPos) {
        StoreData.pwm_vent = StoreData.pwm_vent + 5;

        if (StoreData.pwm_vent > 250) {
          StoreData.pwm_vent = 250;
        }

        lcd.setCursor(0,1);
        lcd.print(int(StoreData.pwm_vent/2.5));
        lcd.print(F(" %"));
        wait_ms(200L);
      }
      oldEncPos = encPos;
    }
  } while(!(encoder.getButton() == 5 ));
  encPos = 1;
  EEPROM.put(EEADDR,StoreData);
  updateMenu();
}
/* set_vent */

#ifdef USE_HUMIDIFIER
void set_humid(){
  lcd.clear();
  lcd.home();
  lcd.print(F("P")); lcd.write(4); lcd.print(F("ra tart:"));
  lcd.setCursor(0,1);
  lcd.print(StoreData.hum);

  do{
    encPos += encoder.getValue();

    if (encPos != oldEncPos) {
      if (encPos < oldEncPos) {
        StoreData.hum =  StoreData.hum - 1;

        if (StoreData.hum < 0) {
          StoreData.hum = 0;
        } 

        lcd.setCursor(0,1);
        lcd.print(StoreData.hum);
        lcd.print(F(" %"));
        wait_ms(200L);
      } else if (encPos > oldEncPos) {
        StoreData.hum = StoreData.hum + 1;

        if (StoreData.hum > 100) {
          StoreData.hum = 100;
        }

        lcd.setCursor(0,1);
        lcd.print(StoreData.hum);
        lcd.print(F(" %"));
        wait_ms(200L);
      }
      oldEncPos = encPos;
    }
  } while(!(encoder.getButton() == 5 ));
  encPos = 1;
  EEPROM.put(EEADDR,StoreData);
  updateMenu();
}
/* set_humid */
#endif

void stop_incubation(){
  #ifdef USE_Triac
    dimmer.setPower(0);
    dimmer.setState(OFF);
  #else
    digitalWrite(heat, LOW);
  #endif
  digitalWrite(vent, LOW);
  digitalWrite(LedErr, LOW);
  digitalWrite(LedRun, LOW);
  #ifdef USE_HUMIDIFIER
    servo1.moveToDegrees(0);
  #endif
  StoreData.runflag = 0;
  EEPROM.put(EEADDR,StoreData);
}
/* stop_incubation */

void run_incubation(){
  digitalWrite(LedRun, HIGH);

  #ifdef USE_Triac
    // triak kimenet engedélyezése
    if ( !dimmer.getState() ) { dimmer.setState(ON); }
  #endif
  // EEprom olvas
  EEPROM.get(EEADDR, StoreData);

  // Vent 
  if (StoreData.pwm_vent >= 35) {
    analogWrite(vent, StoreData.pwm_vent);
  } else {
    digitalWrite(vent, LOW);
  }

  // Hőmérséklet mérés 
  T1 = t_1air.Thermistor_Read();
  T2 = t_2air.Thermistor_Read(); 
  airTemp = ( T1 + T2 ) / 2;
  TH = t_heat.Thermistor_Read();

  sensors_event_t event;

  // Érzékelők hibafigyelése 
  if ( T1 < (-30.0) ){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NTC 1 szakadt"));
    wait_ms(2000L);
    stop_incubation();
  } else if ( T1 > 120.0 ){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NTC 1 zarlat"));
    wait_ms(2000L);
    stop_incubation();
  } else if ( T2 < (-30.0) ){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NTC 2 szakadt"));
    wait_ms(2000L);
    stop_incubation();
  } else if ( T2 > 120.0 ){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NTC 2 zarlat"));
    wait_ms(2000L);
    stop_incubation();
  } else if ( TH < (-30.0) ){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NTC Fu szakadt"));
    wait_ms(2000L);
    stop_incubation();
  } else if ( TH > 120.0 ){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("NTC Fu zarlat"));
    wait_ms(2000L);
    stop_incubation();
    } else { 

    // millis()
    prevTime = Time;
    Time = millis();

    // PID számítás 
    error = (StoreData.kozep - airTemp)*100;  // Hibajel, 100-s szorzás a kis értékek miatt
    changeError = error - last_error;   // Derivált tag
    totalError += error;  // Integrált hiba ()
    if (totalError > 20000) {totalError = 20000;}

    pidTerm = (Kp*error) + (Ki*totalError) + ((Kd*changeError)/(Time-prevTime));  //PID kimenet számítás
    if ( pidTerm > outMax ) pidTerm = outMax;
    else if (pidTerm < outMin ) pidTerm = outMin;
    pwmOut = pidTerm; 
    last_error = error;

    // Fűtés       
    if ( TH < 77.0) {
      #ifdef USE_Triac
        dimmer.setPower(pwmOut);
      #else
        analogWrite(heat, pwmOut);
      #endif
    } else if ( TH >= 80.0) {
      #ifdef USE_Triac
        dimmer.setPower(0);
      #else
        digitalWrite(heat, LOW);
      #endif
      
    }

    // Kijelző 
    lcd.clear();
    lcd.setCursor(0, 0);
    //                   levegő NTC                     levegő beállított
    lcd.print(F("Lev ")); lcd.print(airTemp); lcd.print(F(" C"));lcd.write(1);lcd.print(F("l ")); lcd.print(StoreData.kozep);
    // ~3mp lekérdezés
    if (lcdtime >= 3){
      //double humid = dht.readHumidity();
      dht.humidity().getEvent(&event);
      double humid = event.relative_humidity;
      prevhum = humid;
      lcd.setCursor(0, 1);
      lcd.print(F("P"));lcd.write(160);lcd.print(F("ratart ")); lcd.print(humid);lcd.print(F("%"));
      lcdtime = 0;
      #ifdef USE_HUMIDIFIER
        if (StoreData.hum >= humid + 5) {
          servo1.moveToDegrees(50);
        } else if (StoreData.hum >= humid + 10) {
          servo1.moveToDegrees(90);
        } else if (StoreData.hum <= humid + 5) {
          servo1.moveToDegrees(50);
        } else if (StoreData.hum <= humid + 10) {
          servo1.moveToDegrees(0);
        }
      #endif

    } else  {
      lcd.setCursor(0, 1);
      lcd.print(F("P"));lcd.write(160);lcd.print(F("ratart ")); lcd.print(prevhum);lcd.print(F("%"));
      lcdtime += 1; 
    }
    lcd.setCursor(0, 2);
    //   Vent xx 
    lcd.print(F("Vent "));lcd.print(int(StoreData.pwm_vent/2.5)); lcd.print(F("%"));
    
    #ifdef USE_Triac
      lcd.print(F(" F"));lcd.write(3);lcd.print(F("t ")); lcd.print(pwmOut); lcd.print(F("%"));
    #else
      lcd.print(F(" F"));lcd.write(3);lcd.print(F("t ")); lcd.print(int(pwmOut/2.55)); lcd.print(F("%"));
    #endif
    lcd.setCursor(0, 3); 
    //fűtőtest hőm
    lcd.print(F("F.test ")); lcd.print(TH);
          
    // LED vezérlés
    if ((airTemp >= (StoreData.kozep - StoreData.hyst)) && (airTemp <= (StoreData.kozep + StoreData.hyst))) {
      digitalWrite(LedErr, LOW);
      } else if (airTemp < (StoreData.kozep - StoreData.hyst)) {
      digitalWrite(LedErr, HIGH);
    } else if (airTemp >= (StoreData.kozep + StoreData.hyst)) {
      digitalWrite(LedErr, HIGH);
    }
  } 
  wait_ms(500L);
  buttonState = encoder.getButton();

  
  if (buttonState == 4 ) {
    if (StoreData.runflag  == 0) {
      StoreData.runflag  = 1;
    } else if (StoreData.runflag  == 1) {
      StoreData.runflag  = 0;
      stop_incubation();
      updateMenu();
    }
  }
  encPos = 1;
}
/* run_incubation */

void updateMenu() {
    EEPROM.get(EEADDR, StoreData);
  switch (menu) {
    case 0:
      menu = 1;
      break;
    case 1:
      lcd.clear();
      lcd.setCursor(7,0);
      lcd.print(F("SETUP"));
      lcd.setCursor(0,1);
      lcd.write(246);
      lcd.print(F("C")); lcd.write(1); lcd.print(F("l h"));lcd.write(2); lcd.print(F("m: "));
      lcd.print(StoreData.kozep); lcd.write(178);lcd.print(F("C"));
      lcd.setCursor(0,2);
      lcd.print(F(" Elt")); lcd.write(1); lcd.print(F("r"));lcd.write(1); lcd.print(F("s: "));
      lcd.print(StoreData.hyst); lcd.write(178);lcd.print(F("C"));
      lcd.setCursor(0,3);
      lcd.print(F(" Vent: "));lcd.print(int(StoreData.pwm_vent/2.5)); lcd.print(F("%"));
      break;
    case 2:
      lcd.clear();
      lcd.setCursor(7,0);
      lcd.print(F("SETUP"));
      lcd.setCursor(0,1);
      lcd.print(F(" C")); lcd.write(1); lcd.print(F("l h"));lcd.write(2); lcd.print(F("m: "));
      lcd.print(StoreData.kozep); lcd.write(178);lcd.print(F("C"));
      lcd.setCursor(0,2);
      lcd.write(246);
      lcd.print(F("Elt")); lcd.write(1); lcd.print(F("r"));lcd.write(1); lcd.print(F("s: "));
      lcd.print(StoreData.hyst); lcd.write(178);lcd.print(F("C"));
      lcd.setCursor(0,3);
      lcd.print(F(" Vent: "));lcd.print(int(StoreData.pwm_vent/2.5)); lcd.print(F("%"));
      break;
    case 3:
      lcd.clear();
      lcd.setCursor(7,0);
      lcd.print(F("SETUP"));
      lcd.setCursor(0,1);
      lcd.print(F(" C")); lcd.write(1); lcd.print(F("l h"));lcd.write(2); lcd.print(F("m: "));
      lcd.print(StoreData.kozep); lcd.write(178);lcd.print(F("C"));
      lcd.setCursor(0,2);
      lcd.print(F(" Elt")); lcd.write(1); lcd.print(F("r"));lcd.write(1); lcd.print(F("s: "));
      lcd.print(StoreData.hyst); lcd.write(178);lcd.print(F("C"));
      lcd.setCursor(0,3);
      lcd.write(246);
      lcd.print(F("Vent: "));lcd.print(int(StoreData.pwm_vent/2.5)); lcd.print(F("%"));
      break;
  #ifndef USE_HUMIDIFIER
    case 4:
      menu = 3;
  #else
    case 4:
      lcd.clear();
      lcd.setCursor(7,0);
      lcd.print(F("SETUP"));
      lcd.setCursor(0,1);
      lcd.write(246);
      lcd.print(F("P")); lcd.write(4); lcd.print(F("ra tart:"));
      lcd.print(StoreData.hum); lcd.print(F("%"));
      break;
    case 5:
      menu = 4;
  #endif
  }
}
/* updateMenu */

void executeAction() {
  switch (menu) {
    case 1:
      set_kozep();
      break;
    case 2:
      set_hyst();
      break;
    case 3:
      set_vent();
      break;
  #ifdef USE_HUMIDIFIER
    case 4:
      set_humid();
      break;
  #endif
  }
}
/* executeAction */

void loop() {
  #ifdef USE_HUMIDIFIER
    servo1.update();
  #endif
  #ifdef USE_EGGMOTOR
    servo2.update();
  #endif
  encPos += encoder.getValue();

  if (encPos != oldEncPos) {
    oldEncPos = encPos;
    updateMenu();
  }

  buttonState = encoder.getButton();

  if (buttonState == 5) {
    executeAction();
  } else if ( buttonState == 4 ) {
    if (StoreData.runflag  == 0) {
      StoreData.runflag  = 1;
      EEPROM.put(EEADDR,StoreData);
    } else if (StoreData.runflag  == 1) {
      StoreData.runflag  = 0;
      EEPROM.put(EEADDR,StoreData);
      stop_incubation();
    }
  }

  if (StoreData.runflag == 1) {
     run_incubation();
  }
}
/* loop */

void timerIsr() {
  encoder.service();
}