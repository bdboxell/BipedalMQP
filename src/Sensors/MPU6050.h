#include "../Utilities/MathUtils.h"
#include "Arduino.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


class MPU6050 {
	public:
		MPU6050();
		void init();
		void update();
		void calibrate();
		void get_data(Pose*);
		Pose get_data();
		void print();
		void calibrate_pitch();

	private:
		Pose cur_pose;
		unsigned long lastTimeStamp = millis();
        const float g = 10.1;
        const float fudge_factor = 16.7; //16.3
		float roll_filter[2];
		float pitch_filter[2];

		void get_raw_data(Pose* pose);
		void normalize_data(Pose* data);
		void kalman_filter(float elapsed_s, float k_input, float k_measure, float* output);
		Adafruit_MPU6050 mpu;
	    Pose average_filter;
};