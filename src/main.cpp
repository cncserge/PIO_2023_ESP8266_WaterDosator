

#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
#include "pcf8574_esp.h"
#include "Button.h"
#include "EEPROM.h"
// I2C device found at address 0x27  ! 020
// I2C device found at address 0x38  ! 3ff

#define PCF8574_ADDRESS 0x20
#define NODEMCU_PIN_D1 5	// I2C
#define NODEMCU_PIN_D2 4	// I2C
#define I2C_SCL NODEMCU_PIN_D1
#define I2C_SDA NODEMCU_PIN_D2
LiquidCrystal_I2C lcd(0x27, 16,2, YWROBOT);
PCF8574 pcf8574(PCF8574_ADDRESS,I2C_SDA, I2C_SCL);

uint8_t portInputButton = 0;

bool bt_up;
bool bt_dn;
bool bt_ok;


Button btUp(&bt_up, INPUT_PULLDOWN_16);
Button btDn(&bt_dn, INPUT_PULLDOWN_16);
Button btOk(&bt_ok, INPUT_PULLDOWN_16);
Button btStart(14, INPUT_PULLUP);

const int pinKlapan = 15;
const int pinBuzzer = 16;
void menuPrc(void);
void setupPrc(void);
void isrCount(void);
enum enumMenuMode{SETUP, RUN};
volatile double valSetPoint = 345;
volatile double valCntCurrent = 0;
volatile double pulseCnt;
volatile unsigned long pulseLiter = 40;
volatile long tempCounter = 0;
int menuMode = RUN;





void setup() {
  EEPROM.begin(512);
  //Serial.begin(9600);
  pinMode(pinKlapan, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
#ifdef ON_LOW
  digitalWrite(pinKlapan, HIGH);
#else
  digitalWrite(pinKlapan, LOW);
#endif
  lcd.init();
  lcd.backlight();
  if(EEPROM.read(0) != 8){
    EEPROM.write(0, 8);
    EEPROM.put(10, valSetPoint);
    EEPROM.put(20, pulseLiter);
    EEPROM.commit();
  }
  EEPROM.get(10, valSetPoint);
  EEPROM.get(20, pulseLiter);

  portInputButton = pcf8574.read8();
  if(portInputButton == 254) bt_dn = true;  else bt_dn = false;
  if(portInputButton == 253) bt_ok = true;  else bt_ok = false;
  if(portInputButton == 251) bt_up = true;  else bt_up = false;

  while(btOk.readState()){
  portInputButton = pcf8574.read8();
  if(portInputButton == 254) bt_dn = true;  else bt_dn = false;
  if(portInputButton == 253) bt_ok = true;  else bt_ok = false;
  if(portInputButton == 251) bt_up = true;  else bt_up = false;


    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Release button");
    menuMode = SETUP;
    delay(500);
  }
  pinMode(12, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
}

void loop(){

  if(menuMode == RUN){
#ifdef ON_LOW
    //if(digitalRead(pinKlapan) == HIGH){
#else
    if(digitalRead(pinKlapan) == LOW){
#endif
      if(btUp.isBlinkPressed() || btUp.isPressed()){
        if(valSetPoint < 20000.)
          valSetPoint ++;
      }
      if(btDn.isBlinkPressed() || btDn.isPressed()){
        if(valSetPoint > 1)
          valSetPoint --;
      }
      if(btOk.isPressed() || btStart.isPressed()){
        valCntCurrent = 0;
        pulseCnt = 0;
        tempCounter = 0;
#ifdef ON_LOW
        digitalWrite(pinKlapan, LOW);
#else
        digitalWrite(pinKlapan, HIGH);
#endif
        int t = 0;
        EEPROM.get(10, t);
        if(t != valSetPoint){
          EEPROM.put(10, valSetPoint);
          EEPROM.commit();
          // Serial.println(valSetPoint);
        }
        attachInterrupt(digitalPinToInterrupt(12), isrCount, CHANGE);
      }
    }
    else{
      if(btOk.isPressed() || btStart.isPressed()){
#ifdef ON_LOW
        digitalWrite(pinKlapan, HIGH);
#else
        digitalWrite(pinKlapan, LOW);
#endif
        detachInterrupt(digitalPinToInterrupt(12));
      }
    }
  }

  else if(menuMode == SETUP){
      if(btUp.isBlinkPressed() || btUp.isPressed()){
        if(pulseLiter < 10000)
          pulseLiter ++;
      }
      if(btDn.isBlinkPressed() || btDn.isPressed()){
        if(pulseLiter > 1)
          pulseLiter --;
      }
      if(btOk.isLongPressed()){
        EEPROM.put(20, pulseLiter);
        EEPROM.commit();
        menuMode = RUN;
      }
  }
  portInputButton = pcf8574.read8();
  if(portInputButton == 254) bt_dn = true;  else bt_dn = false;
  if(portInputButton == 253) bt_ok = true;  else bt_ok = false;
  if(portInputButton == 251) bt_up = true;  else bt_up = false;
  btUp.run();
  btDn.run();
  btOk.run();
  btStart.run();
  menuPrc();
  setupPrc();


  {
    static unsigned long tim = millis();
    static bool buzz = false;
    if(btUp.isPressed() || btDn.isPressed() || btOk.isPressed() || btStart.isPressed()){
      buzz = true;
    }
    if(buzz){
      digitalWrite(pinBuzzer, HIGH);
      if(millis() - tim >= 100UL){
        buzz = false;
      }
    }
    else{
      digitalWrite(pinBuzzer, LOW);
      tim = millis();
    }
  }
  // {
  //   static unsigned long t = millis();
  //   if(millis() - t >= 500){
  //     t = millis();
  //     Serial.println(" pulseCnt      :" + String(pulseCnt));
  //     Serial.println(" valCntCurrent :" + String(valCntCurrent));
  //     Serial.println(" tempCounter   :" + String(tempCounter));
  //     Serial.println("------------------");
  //     Serial.println("------------------");
  //     Serial.println("------------------");
  //     Serial.println("------------------");
  //     Serial.println("------------------");
  //   }
  // }
}


void setupPrc(void){
  static unsigned long preValpulseLiter = -1;
  static int preMenuMode = -1;
  if(preMenuMode != menuMode){
    preMenuMode = menuMode;
    if(menuMode == SETUP){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("  puls per divi ");
    }
  }
  if(menuMode == SETUP){
    if(preValpulseLiter != pulseLiter){
      preValpulseLiter = pulseLiter;
      lcd.setCursor(6, 1);
      lcd.print("     ");
      lcd.setCursor(6, 1);
      lcd.print(pulseLiter);
    }
  }
  else{
    preValpulseLiter = -1;
    preMenuMode = -1;
  }
}



void menuPrc(void){
  static int preValSetPoint = -1;
  static int prepulseCnt = -1;
  static int preMenuMode = -1;
  static int preClapan = -1;
  int        clapan = 0;
  #ifdef ON_LOW // end -> close state
            clapan = (digitalRead(pinKlapan) == HIGH ? 0 : 1);
            //digitalWrite(pinKlapan, HIGH);
  #else
            //digitalWrite(pinKlapan, LOW); 
            clapan = (digitalRead(pinKlapan) == LOW ? 0 : 1);
  #endif 

  if(preMenuMode != menuMode){
    preMenuMode = menuMode;
    if(menuMode == RUN){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set point:");
      lcd.setCursor(0, 1);
      lcd.print("Filled   :");
    }
  }
  if(preClapan != clapan){
    preClapan = clapan;
    if(menuMode == RUN){
      lcd.setCursor(7, 1);
      lcd.print("  ");
      lcd.setCursor(7, 1);
      if(clapan)
        lcd.print("->");
      else
        lcd.print("-|");
    }
  }
  if(menuMode == RUN){
    if(preValSetPoint != valSetPoint){
      preValSetPoint = valSetPoint;
      lcd.setCursor(10, 0);
      lcd.print("     ");
      lcd.setCursor(10, 0);
      lcd.print((double)valSetPoint * 0.005, 3);
    }
    if(prepulseCnt != (int)pulseCnt){
      prepulseCnt = (int)pulseCnt;
      lcd.setCursor(10, 1);
      lcd.print("     ");
      lcd.setCursor(10, 1);
      lcd.print(pulseCnt * 0.001, 3);
    }
  }
  else{
    preValSetPoint = -1;
    preMenuMode = -1;
  }
}
ICACHE_RAM_ATTR void isrCount(void){
        tempCounter++;
        pulseCnt += (1000. / pulseLiter);
        valCntCurrent = valSetPoint * 5.0;
        if(pulseCnt >= valCntCurrent){
#ifdef ON_LOW
            digitalWrite(pinKlapan, HIGH);
#else
            digitalWrite(pinKlapan, LOW); 
#endif 
        }
}




/*

 // --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
#include <Arduino.h>
#include <Wire.h>


void setup()
{
  

  Serial.begin(9600);
  Serial.println("\nI2C Scanner");
  Serial.println("\nI2C Scanner");
  Serial.println("\nI2C Scanner");
  Serial.println("\nI2C Scanner");
  Serial.println("\nI2C Scanner");
  Serial.println("\nI2C Scanner");
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");

  Wire.begin();
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

*/