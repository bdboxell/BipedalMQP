#include "Arduino.h"
#include "IMU.h"

class PWMMonitor {
    public:
        PWMMonitor();
        int get_wavelength();
        void init(int p, void* imu_ref);
        static void* PWM_obj;

    private:
        int wavelength = 0;
        unsigned long last_time = micros();
        void rising_edge();
        void falling_edge();
        static void rising_edge_ISR();
        static void falling_edge_ISR();
        static int pin;
        static bool is_high;
        void* IMU_obj;
};