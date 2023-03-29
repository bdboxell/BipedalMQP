// #include <Teensy_ADIS16470.h>
#include <ADIS16470.h>
#include <SPI.h>
#include <Wire.h>
#include <MathUtils.h>
#include <Arduino.h>

#define cs_pin 5
#define dr_pin 17
#define rst_pin 16

class IMU {
    public:
        IMU();
        ~IMU();
        void init();
        void calibrate();
        void get_data(Pose* pose);
        Pose get_data();
        void calibrate_pitch();
        static void* IMU_obj; 
    private:
        static void ISR();
        void grabData();
        void scaleData();
        uint16_t *burstData;
        int16_t burstChecksum = 0;

        //Accelerometer and Gyro Values
        float AXS, AYS, AZS = 0;
        float GXS, GYS, GZS = 0;
        // Control registers
        int MSC = 0;
        int FLTR = 0;
        int DECR = 0;

        // Temperature
        float TEMPS = 0;

        ADIS16470 imu = ADIS16470(cs_pin, dr_pin, rst_pin);
};