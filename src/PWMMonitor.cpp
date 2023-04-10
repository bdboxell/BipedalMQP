#include "PWMMonitor.h"

void* PWMMonitor::PWM_obj = nullptr;
bool PWMMonitor::is_high = false;
int PWMMonitor::pin = 0;

PWMMonitor::PWMMonitor() {
    // PWM_obj = this;
}

void PWMMonitor::init(int p) {
    pin = p;   
    PWM_obj = this;
    Serial.println("PWM_Obj Updated!");

    pinMode(pin, INPUT);
    attachInterrupt(p, falling_edge_ISR, CHANGE);
}

void PWMMonitor::rising_edge() {
    last_time = micros();
}

void PWMMonitor::falling_edge() {
    wavelength = micros() - last_time;
}

void PWMMonitor::rising_edge_ISR() {
    Serial.println("Rising!");
    ((PWMMonitor*) PWM_obj)->rising_edge();
}

void PWMMonitor::falling_edge_ISR() {
    if (!digitalRead(pin)) {
        // Serial.println("Falling");
        ((PWMMonitor*) PWM_obj)->falling_edge();
    }
    else {
        // Serial.println("Rising!");
        ((PWMMonitor*) PWM_obj)->rising_edge();
    }
    is_high = !is_high;
}

int PWMMonitor::get_wavelength() {
    return wavelength;
}