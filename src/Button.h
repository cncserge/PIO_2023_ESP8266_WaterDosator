#ifndef BUTTON_H_
#define BUTTON_H_
#include <Arduino.h>

class Button
{
protected:
    const bool *value;
    int pin, cntBlink;
    uint8_t mode;
    unsigned long timCycleMs;
    bool readVal, pressed, released, blink;
	unsigned int countLongPressed = 0;
public:
    explicit Button(int pin, uint8_t mode = INPUT_PULLUP, unsigned long timCycleMs = 100UL);
    explicit Button(const bool *const val, uint8_t mode = INPUT_PULLUP, unsigned long timCycleMs = 100UL);
    void run(void);                              // запускаем в лупе
    bool isPressed(void) { return pressed; };    // если был факт нажатия
    bool isReleased(void) { return released; };  // если был факт отжатия
    bool getState(void) { return readVal; };     // состояние кнопки считаное в лупе
    bool readState(void);                        // состояние кнопки считаное сейчас
    bool isBlinkPressed(void) { return blink; }; // моргаем чем дольше тем больше возвращаемое значени
	bool isLongPressed(const unsigned int cnCycle = 15);       // если долго держем
private:
    bool isValueNoRead;
    bool preReadVal;
    unsigned long timRead, timBlink;
};
Button::Button(int pin, uint8_t mode, unsigned long timCycleMs) : pin(pin),
                                                                  mode(mode),
                                                                  timCycleMs(timCycleMs)
{
    if (this->mode == INPUT_PULLUP)
    {
        pinMode(this->pin, INPUT_PULLUP);
    }
    isValueNoRead = false;
}

Button::Button(const bool *const val, uint8_t mode, unsigned long timCycleMs) : value(val),
                                                                                mode(mode),
                                                                                timCycleMs(timCycleMs)
{
    isValueNoRead = true;
}


bool Button::isLongPressed(const unsigned int cnCycle){
    if(countLongPressed == 999) return false;
	if(countLongPressed > cnCycle){
        countLongPressed = 999;
		return true;
	}
	else{
		return false;
	}
}


void Button::run(void)
{
    pressed = false;
    released = false;
    blink = false;
    if (millis() - timRead >= timCycleMs)
    {
        timRead = millis();
        readVal = readState();
        if (readVal && cntBlink < 100){
            cntBlink++;
		}
        if (preReadVal != readVal)
        {
            if (readVal == true && preReadVal == false)
                pressed = true;
            if (readVal == false && preReadVal == true)
                released = true;
        }
        preReadVal = readVal;
		if(readVal && (countLongPressed < 999)){
             countLongPressed++;
        }
    }
    if (readVal)
    {
        unsigned long t;
        if (cntBlink < 10)
            t = 500;
        else if (cntBlink >= 10 && cntBlink < 20)
            t = 250;
        else if (cntBlink >= 20 && cntBlink < 30)
            t = 150;
        else if (cntBlink >= 30 && cntBlink < 40)
            t = 100;
        else if (cntBlink >= 40)
            t = 10;
        if (millis() - timBlink >= t)
        {
            timBlink = millis();
            blink = true;
        }
		
    }
    else
    {
        cntBlink = 0;
        timBlink = millis();
		countLongPressed = 0;
    }
}
bool Button::readState(void)
{
    int temp;
    if(isValueNoRead == true) temp = *value;
    else                      temp = digitalRead(pin);

    if (temp == HIGH)
    {
        if (mode == INPUT_PULLUP)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        if (mode == INPUT_PULLUP)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
#endif
