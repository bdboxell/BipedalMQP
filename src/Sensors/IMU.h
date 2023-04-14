// #include <Teensy_ADIS16470.h>
#pragma once

#include "ADIS16470.h"
#include <SPI.h>
#include <Wire.h>
#include "../Utilities/MathUtils.h"
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
        void updateData();
        static void* IMU_obj; 
        void reset();
        Pose raw_data;
        Pose cur_pose;
        unsigned long lastTimeStamp = millis();
        void adjust_offset(double value);
        void remove_interrupt();
        void update();
        void enable_interrupt();

    private:
        static void ISR();
        void grabData();
        void scaleData();
        void normalize_data(Pose* data);
        uint16_t *burstData;
        int16_t burstChecksum = 0;

        //Accelerometer and Gyro Values

        // Control registers
        int MSC = 0;
        int FLTR = 0;
        int DECR = 0;

        // Temperature
        float TEMPS = 0;

	    Pose average_filter;

        ADIS16470 imu = ADIS16470(cs_pin, dr_pin, rst_pin);
        double roll_filter[2];
		double pitch_filter[2];
        void kalman_filter(double elapsed_s, double k_input, double k_measure, double* output);

        double pitch_offset = 9.45; //9.45 for level ground, 10.05 for desk
        
};