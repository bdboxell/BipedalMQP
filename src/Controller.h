#include <Arduino.h>
#include "PWMMonitor.h"

class Controller {
    public:
        Controller(int t_pin, int s_pin);
        float get_throttle();
        float get_steering();
        void init(void* imu_ref);

    private:
        PWMMonitor thr_monitor = PWMMonitor();
        // PWMMonitor str_monitor = PWMMonitor();

        const float ms_min = 1000;
        const float ms_max = 2000;
        int throttle_pin;
        int steering_pin;
        inline float scale(int input);
};