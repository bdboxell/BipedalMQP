#include "Arduino.h"
#include "IMU.h"

class PWMMonitor {
    public:
        PWMMonitor();
        int get_wavelength();
        void init(int p);
        static void* PWM_obj;
        static void* PWM_obj_2;
        int pin;


    private:
        int wavelength = 0;
        unsigned long last_time = micros();
        void rising_edge();
        void falling_edge();
        static void ISR();
        static void ISR_2();
};