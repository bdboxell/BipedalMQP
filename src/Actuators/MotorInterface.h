#pragma once

#include <Arduino.h>
#include "ESP32Servo.h"

class MotorInterface {
    private:
        int msMin = 1200;
        int msMax = 1800;
        int center = (msMin + msMax) /2;
        int setInterval = center;
        unsigned long lastIntervalStart = 0;
        unsigned long nextDutyEnd = 0;
        unsigned long nextIntervalEnd = 0;
        int pin;
        float deadband_upper = 4;
        float deadband_lower = 4;
        float expo = 1.5;
        bool reversed = false;
        Servo pwmOut;
        
    public:
        MotorInterface(int pin);
        MotorInterface(int pin, bool rev);
        MotorInterface(int pin, bool rev, int ms_min, int ms_max);
        int write_percent(float pct);
        int update();
        void stop();
};