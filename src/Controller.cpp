#include "Controller.h"
#include <Arduino.h>

Controller::Controller(int t_pin, int s_pin) {
    throttle_pin = t_pin;
    steering_pin = s_pin;

    pinMode(throttle_pin, INPUT);
    pinMode(steering_pin, INPUT);
}

float Controller::get_steering() {
    int pulse = pulseIn(steering_pin, HIGH);
    float scaled = scale(pulse);
    scaled = (fabs(scaled) < 1)? 0: scaled;
    return scaled;
}
float Controller::get_throttle() {
    int pulse = pulseIn(throttle_pin, HIGH);
    float scaled = scale(pulse);
    scaled = (fabs(scaled) < 1)? 0: scaled;
    return scaled;
}

inline float Controller::scale(int input) {
    return 200*(input - ms_min)/(ms_max-ms_min)-100;
}