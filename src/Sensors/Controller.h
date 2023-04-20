#include <Arduino.h>
#include "PWMMonitor.h"

class Controller {
    public:
        Controller(int t_pin, int s_pin, int a_pin);
        float get_throttle();
        float get_steering();
        float get_aux();
        void init();

    private:
        PWMMonitor thr_monitor = PWMMonitor();
        PWMMonitor str_monitor = PWMMonitor();
        PWMMonitor aux_monitor= PWMMonitor();

        const float ms_min = 1100;
        const float ms_max = 1900;
        int throttle_pin;
        int steering_pin;
        int aux_pin;
        inline float scale(int input);
};