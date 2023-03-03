#include "MotorInterface.h"

MotorInterface::MotorInterface(int pinNum) {
    pin = pinNum;
    pinMode(pin,OUTPUT);
    pwmOut.attach(pinNum, msMin, msMax);
}

MotorInterface::MotorInterface(int pinNum, bool rev) {
    pin = pinNum;
    reversed = rev;
    pinMode(pin,OUTPUT);
    pwmOut.attach(pinNum, msMin, msMax);
}

int MotorInterface::write_percent(float pct) {

    pct = ((reversed)? -1 : 1) *pct;

    // make sure the result is between -100 and 100
    pct = (pct>100)? 100: ((pct<-100)? -100: pct);

    // apply deadband. If pct is closer to deadband threshhold than 0, round up to the threshhold
    if (pct>0) pct = (pct/100.0)*(100-deadband_upper) + deadband_upper;
    if (pct<0) pct = (pct/100.0)*(100-deadband_lower) - deadband_lower;

    // convert to PWM wavelength
    setInterval = ((pct+100)/200.0)*(msMax - msMin) + msMin;

    pwmOut.writeMicroseconds(setInterval);
    return setInterval;
}

void MotorInterface::stop() {
    write_percent(0);
}