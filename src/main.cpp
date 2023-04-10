#include <Arduino.h>
#include "MotorInterface.h"
#include "MPU6050.h"
#include "IMU.h"
#include "pid.h"
#include "WebClient.h"
#include "SPIFFS.h"
#include "DataPacket.h"
#include "Controller.h"
// #include "MathUtils.h"

MotorInterface right_motor = MotorInterface(14, false);
MotorInterface left_motor = MotorInterface(12, true);
IMU imu;
// Controller controller = Controller(35,34);

//oscillation at kp = 3, ki = 0.01

PID balance_pid = pid_init(45, 800, 3.2, 0.015, 0.75, 99999); //This was used in the youtube short
// PID balance_pid = pid_init(40, 825, 3.1, 0.015, 0.75, 99999);

String serial_in = "";

WebClient web_client;

bool logging = false;

void balance();
void input();

float last_power = 0;

int iter_count = 0;
bool active = false;

void setup(void) {


  Serial.begin(115200);
  delay(300);
  imu.init();
  imu.calibrate();
  // imu.remove_interrupt();
  delay(500);
  imu.reset();
  // imu.calibrate_pitch();
  // controller.init();
}

void loop() {
  // imu.update();
  if (iter_count>5) {
    Pose pose = imu.get_data();
    // Serial.print(pose.pitch);
    // Serial.print(", ");
    // Serial.print(pose.roll);
    // Serial.print(", ");
    // Serial.print(pose.yaw);
    // Serial.print(", ");
    // Serial.print(pose.x);
    // Serial.print(", ");
    // Serial.print(pose.y);
    // Serial.print(", ");
    // Serial.println(pose.z);

    iter_count = 0;
  }
  iter_count++;
  // imu.print();
  input();
  Pose pose = imu.get_data();
  if (!active && fabs(pose.pitch) < 0.1)
    active = true;
  if (active)
    balance();
  // float out = controller.get_steering();
  // Serial.println(controller.get_throttle());
  // Serial.print(", ");
  // Serial.println(controller.get_throttle());
  delay(2);
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
  // left_input = (controller.get_steering()) * input_gain;
  // right_input = (controller.get_steering()) * input_gain;

  left_input = 0;
  right_input = 0;

  // float target_angle = -controller.get_throttle()*0.08;

  // float power_addend = controller.get_throttle()*0.1;
  float target_angle = 0;

  Pose pose = imu.get_data();

  float max = 100;

  if(serial_in == "w") {
    target_angle = 2;
     Serial.print("\nPower: ");
    Serial.println(target_angle);
  }
  else if (serial_in == "s") {
    logging = !logging;
  }
  else if (serial_in == "u") {
    Serial.println("Biasing more forward!");
    imu.adjust_offset(0.05);
  }
  else if (serial_in == "d") {
    Serial.println("Biasing more backward!");
    imu.adjust_offset(-0.05);
  }
  
  if (fabs(pose.pitch) < 20) {
    double com_displace = 4.5*sin(pose.pitch*DEG_TO_RAD);
    float power = pid_calculate(&balance_pid, target_angle, com_displace);
    float exponent = 1;
    power = ((power < 0)? -1: 1)*fabs(pow(power/100, exponent))*100;
    // power += power_addend;
    // last_power = power;

    float left_power = power-left_input;
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