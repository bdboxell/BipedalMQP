#include <Arduino.h>
#include "Actuators/MotorInterface.h"
#include "Sensors/MPU6050.h"
#include "Sensors/IMU.h"
#include "Controls/pid.h"
#include "Utilities/WebClient.h"
#include "SPIFFS.h"
#include "Utilities/DataPacket.h"
#include "Sensors/Controller.h"
#include "Controls/Balance.h"

IMU imu;
Controller controller = Controller(35,32);
Balance balance_control = Balance();

void handle_input();

bool active = false;

void setup(void) {
  Serial.begin(115200);
  delay(300);
  imu.init();
  imu.calibrate();
  delay(500);
  imu.reset();
}

void loop() {
  // Initialize a new data packet for data logging
  DataPacket packet;

  // Handle inputs from keyboard
  handle_input();

  // Run balance controller if applicable
  Pose pose = imu.get_data();
  if (!active && fabs(pose.pitch) < 0.1)
    active = true;
  if (active && fabs(pose.pitch) < 20) {
    balance_control.balance(&pose, &packet);
  }
  else {
    balance_control.reset();
  }
  
  //delay to maintain loop integrity
  delay(2); //absolutely has to be 2ms update or PID breaks
}

void handle_input() {
  string serial_in = "";
  while (Serial.available()) {
    delay(3);
    char c = Serial.read();
    serial_in += c;
  }

  // WASD Keyboard Controls to drive
  if(serial_in == "w") {
    Serial.println("Forward!");
    balance_control.set_target_speed(7);
  }
  else if(serial_in == "s") {
    Serial.println("Backward!");
    balance_control.set_target_speed(-7);
  }
  else if(serial_in == "a") {
    Serial.println("Left!");
    balance_control.add_turn_power(-1.5,1.5);
  }
  else if(serial_in == "d") {
    Serial.println("Right!");
    balance_control.add_turn_power(1.5,-1.5);
  }
  // X key to hard stop the robot
  else if(serial_in == "x") {
    Serial.println("Stop!");
    balance_control.stop();
  }
  // L Key to enable/disable logging
  else if (serial_in == "l") {
    balance_control.toggle_logging();
  }
  // Compensation for poor IMU calibration
  // P to make the robot bias forward
  else if (serial_in == "p") {
    Serial.println("Biasing more forward!");
    imu.adjust_offset(0.05);
  }
  // O to make the robot bias backwards
  else if (serial_in == "o") {
    Serial.println("Biasing more backward!");
    imu.adjust_offset(-0.05);
  }
}