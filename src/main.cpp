#include <Arduino.h>
#include "MotorInterface.h"
#include "MPU6050.h"
#include "pid.h"
#include "WebClient.h"
#include "SPIFFS.h"
#include "DataPacket.h"
#include "Controller.h"
// #include "MathUtils.h"

MotorInterface right_motor = MotorInterface(14, true);
MotorInterface left_motor = MotorInterface(12, false);
MPU6050 imu;
Controller controller = Controller(33,32);

//oscillation at kp = 3, ki = 0.01
// PID balance_pid = pid_init(1, 0.04, 150);
PID balance_pid = pid_init(4, 0.025, 0.25);


String serial_in = "";

WebClient web_client;

bool logging = false;

void balance();
void input();

float last_power = 0;

int iter_count = 0;

void setup(void) {


  Serial.begin(115200);
  delay(300);
  imu.init();
  imu.calibrate();
  delay(100);
  imu.calibrate_pitch();
}

void loop() {
  imu.update();
  if (iter_count>5) {
    Pose pose = imu.get_data();
    Serial.println(pose.pitch);
    iter_count = 0;
  }
  iter_count++;
  // imu.print();
  // // input();
  balance();
  // Serial.println(controller.get_steering());
  // Serial.println(controller.get_throttle());
  delay(10);
}

void input() {
  serial_in = "";
  while (Serial.available()) {
    delay(3);
    char c = Serial.read();
    serial_in += c;
  }
}


void calibrate_power() {

}

void balance() {
 
  float left_input, right_input = 0;
  float input_gain = 0.4;
  left_input = (controller.get_steering()) * input_gain;
  right_input = (controller.get_steering()) * input_gain;

  float target_angle = -controller.get_throttle()*0.08;

  Pose pose = imu.get_data();

  float max = 50;

  if(serial_in == "w") {
    target_angle = 2;
     Serial.print("\nPower: ");
    Serial.println(target_angle);
  }
  else if (serial_in == "s") {
    logging = !logging;
  }
  
  if (fabs(pose.pitch) < 20) {
    float power = pid_calculate(&balance_pid, target_angle, pose.pitch);
    float exponent = 1;
    power = ((power < 0)? -1: 1)*fabs(pow(power/100, exponent))*100;
    // last_power = power;

    float left_power = power+left_input;
    float right_power = power+right_input;

    if (left_power>max) {
      left_power = max;
    }
    else if (left_power < -max) {
      left_power = -max;
    }
    if (right_power>max) {
      right_power = max;
    }
    else if (right_power < -max) {
      right_power = -max;
    }

    int pwm = right_motor.write_percent(right_power);
    left_motor.write_percent(left_power);

    if(logging) {
      DataPacket packet;
      packet.add_data("pitch", pose.pitch);
      packet.add_data("power", power);
      packet.add_data("pwm", pwm);

      packet.write_to_serial();
    }
  }
  else {
    reset_pid(&balance_pid);
    left_motor.stop();
    right_motor.stop();
  }
}