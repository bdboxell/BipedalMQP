#include "Controller.h"

Controller::Controller(int t_pin, int s_pin, int a_pin) {
    throttle_pin = t_pin;
    steering_pin = s_pin;
    aux_pin = a_pin;
}

void Controller::init() {
    Serial.println("Controller init");
    str_monitor.init(steering_pin);
    thr_monitor.init(throttle_pin);
    aux_monitor.init(aux_pin);
}

float Controller::get_steering() {
    int pulse = str_monitor.get_wavelength();
    float scaled = scale(pulse);
    scaled = (fabs(scaled) < 1)? 0: scaled;
    return scaled;
}
float Controller::get_throttle() {
    int pulse = thr_monitor.get_wavelength();
    float scaled = scale(pulse);
    scaled = (fabs(scaled) < 1)? 0: scaled;
    return scaled;
}

float Controller::get_aux() {
    int pulse = aux_monitor.get_wavelength();
    float scaled = scale(pulse);
    scaled = (fabs(scaled) < 1)? 0: scaled;
    return scaled;
}

inline float Controller::scale(int input) {
    return 200*(input - ms_min)/(ms_max-ms_min)-100;
}