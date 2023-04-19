#include "PWMMonitor.h"

void* PWMMonitor::PWM_obj = nullptr;
void* PWMMonitor::PWM_obj_2 = nullptr;

PWMMonitor::PWMMonitor() {
    // PWM_obj = this;
}

void PWMMonitor::init(int p) {
    pin = p;
    Serial.println("PWM_Obj Updated!");
    pinMode(pin, INPUT);

    if (PWM_obj == nullptr) {
        PWM_obj = this;
        attachInterrupt(p, ISR, CHANGE);
    }
    else {
        PWM_obj_2 = this;
        attachInterrupt(p, ISR_2, CHANGE);
    }
}

void PWMMonitor::rising_edge() {
    last_time = micros();
}

void PWMMonitor::falling_edge() {
    wavelength = micros() - last_time;
}

void PWMMonitor::ISR() {
    if (!digitalRead(((PWMMonitor*) PWM_obj)->pin)) {
        ((PWMMonitor*) PWM_obj)->falling_edge();
    }
    else {
        ((PWMMonitor*) PWM_obj)->rising_edge();
    }
}

void PWMMonitor::ISR_2() {
    if (!digitalRead(((PWMMonitor*) PWM_obj_2)->pin)) {
        ((PWMMonitor*) PWM_obj_2)->falling_edge();
    }
    else {
        ((PWMMonitor*) PWM_obj_2)->rising_edge();
    }
}

int PWMMonitor::get_wavelength() {
    return wavelength;
}