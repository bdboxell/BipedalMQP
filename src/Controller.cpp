#include "Controller.h"

Controller::Controller(int t_pin, int s_pin) {
    throttle_pin = t_pin;
    steering_pin = s_pin;
}

void Controller::init(void* imu_ref) {
    Serial.println("Controller init");
    // str_monitor.init(steering_pin);
    thr_monitor.init(throttle_pin, imu_ref);
}

float Controller::get_steering() {
    int pulse = pulseIn(steering_pin, HIGH);
    // int pulse = str_monitor.get_wavelength();
    float scaled = scale(pulse);
    scaled = (fabs(scaled) < 1)? 0: scaled;
    return scaled;
}
float Controller::get_throttle() {
    // int pulse = pulseIn(throttle_pin, HIGH);
    int pulse = thr_monitor.get_wavelength();
    // float scaled = scale(pulse);
    // scaled = (fabs(scaled) < 1)? 0: scaled;
    // return scaled;
    return pulse;
}

inline float Controller::scale(int input) {
    return 200*(input - ms_min)/(ms_max-ms_min)-100;
}