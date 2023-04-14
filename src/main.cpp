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
Controller controller = Controller(35,32);

//oscillation at kp = 3, ki = 0.01

// PID balance_pid = pid_init(45, 800, 3.2, 0.015, 0.75, 99999); //This was used in the youtube short
PID balance_pid = pid_init(45, 1000, 3.8, 0.015, 0.75, 99999); //Demo'd to Agheli
// PID balance_pid = pid_init(120, 875, 0.625, 0.015, 0.75, 99999); //P Heavy Testing , Ku = 125



String serial_in = "";

WebClient web_client;

bool logging = false;

void balance();
void input();
double average_speed(double new_val);

float last_power = 0;

int iter_count = 0;
bool active = false;
float target_speed = 0;
float target_angle = 0;
float left_input, right_input = 0;
const int filter_size = 50;
double velocity_filter[filter_size];
int control_state = 0; //0 stopped, -1 backwards, 1 forwards

void setup(void) {


  Serial.begin(115200);
  delay(300);
  // controller.init(&imu);
  imu.init();
  imu.calibrate();
  // imu.remove_interrupt();
  delay(500);
  imu.reset();
  // imu.calibrate_pitch();
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
  if (active) {
    balance();
  }
  // float out = controller.get_steering();
  // Serial.println(controller.get_throttle());
  // Serial.print(", ");
  // Serial.println(controller.get_throttle());

  delay(2); //absolutely has to be 2ms update or PID breaks
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
 
  float input_gain = 0.4;
  // left_input = (controller.get_steering()) * input_gain;
  // right_input = (controller.get_steering()) * input_gain;

  float power_addend = 0;

  // float target_angle = -controller.get_throttle()*0.08;

  // float power_addend = controller.get_throttle()*0.1;

  Pose pose = imu.get_data();

  float max = 100;

  if(serial_in == "w") {
    Serial.println("Forward!");
    target_speed = 7;
    control_state = 1;
  }
  else if(serial_in == "s") {
    Serial.println("Backward!");
    target_speed = -5;
    control_state = -1;
  }
  else if(serial_in == "a") {
    Serial.println("Left!");
    left_input-=2;
    right_input+=2;
  }
  else if(serial_in == "d") {
    Serial.println("Right!");
    left_input+=2;
    right_input-=2;
  }
  else if(serial_in == "x") {
    Serial.println("Stop!");
    left_input= 0;
    right_input= 0;
    power_addend= 0;
    target_angle = 0;
    target_speed = 0;
    control_state = 0;
  }
  else if (serial_in == "l") {
    logging = !logging;
  }
  else if (serial_in == "p") {
    Serial.println("Biasing more forward!");
    imu.adjust_offset(0.05);
  }
  else if (serial_in == "o") {
    Serial.println("Biasing more backward!");
    imu.adjust_offset(-0.05);
  }
  
  if (fabs(pose.pitch) < 20) {
    double com_displace = 4.5*sin(pose.pitch*DEG_TO_RAD);
    float power = pid_calculate(&balance_pid, target_angle, com_displace) - 3000*sin(target_angle*DEG_TO_RAD);
    
    float exponent = 1;
    power = ((power < 0)? -1: 1)*fabs(pow(power/100, exponent))*100;

    double speed = average_speed(power);
    if (control_state == 1 && speed > target_speed) {
      // Serial.println("Stabilizing!");
      target_angle = -0.04;
      power_addend = target_speed;
    }
    else if (control_state ==1 && speed < target_speed) {
      target_angle = -0.08;
      power_addend = 0;
    }

    if (control_state == -1 && speed < target_speed) {
      // Serial.println("Stabilizing!");
      target_angle = 0.03;
      power_addend = target_speed;
    }
    else if (control_state == -1 && speed > target_speed) {
      target_angle = 0.09;
      power_addend = 0;
    }
    
    // else if (target_angle == 0 && power_addend!=0) {
    //   target_angle+=1;
    // }

    power += power_addend;
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

    if(logging&&iter_count>5) {
      DataPacket packet;
      packet.add_data("pitch", pose.pitch);
      packet.add_data("power", power);
      packet.add_data("target_angle", target_angle);

      packet.write_to_serial();
    }
  }
  else {
    reset_pid(&balance_pid);
    left_motor.stop();
    right_motor.stop();
  }
}

double average_speed(double new_val) {
    double sum = 0;
    for (int i = 0 ; i< filter_size - 1; i++) {
        velocity_filter[i] = velocity_filter[i+1];
        sum+=velocity_filter[i];
    }
    sum+=new_val;
    velocity_filter[filter_size-1] = new_val;
    return sum/filter_size;
}