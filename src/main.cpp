#include <Arduino.h>
#include "MotorInterface.h"
#include "MPU6050.h"
#include "pid.h"
#include "WebClient.h"
#include "SPIFFS.h"
#include "DataPacket.h"
// #include "MathUtils.h"

MotorInterface right_motor = MotorInterface(12, true);
MotorInterface left_motor = MotorInterface(14, false);
MPU6050 imu;

//oscillation at kp = 5.25, 4 0's before a digit for I
PID balance_pid = pid_init(12, 0, 1000, 0, 4);

String serial_in = "";

WebClient web_client;

bool logging = false;

void balance();
void input();

float last_power = 0;

void setup(void) {

  
  Serial.begin(115200);
  delay(300);
  imu.init();
  imu.calibrate();
  delay(100);
  imu.calibrate_pitch();

if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  File file = SPIFFS.open("/text.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();

  // web_client.init_server();
}

void loop() {
  imu.update();
  // imu.print();
  input();
  balance();

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
 
  Pose pose = imu.get_data();

  float max = 100;

  float target_angle = 0;
  if(serial_in == "w") {
    target_angle = 2;
     Serial.print("\nPower: ");
    Serial.println(target_angle);
  }
  else if (serial_in == "s") {
    logging = !logging;
  }
  
  if (fabs(pose.pitch) < 15) {
    float power = pid_calculate(&balance_pid, target_angle, pose.pitch);

    float exponent = 1;
    power = ((power < 0)? -1: 1)*fabs(pow(power/100, exponent))*100;

    if (power>max) {
      power = max;
    }
    else if (power < -max) {
      power = -max;
    }
 
    int pwm = right_motor.write_percent(power);
    left_motor.write_percent(power);

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