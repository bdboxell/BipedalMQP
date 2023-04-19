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
#include "Controls/LegControl.h"

IMU imu;
Controller controller = Controller(35,34);
Balance balance_control = Balance();
LegControl legs = LegControl();

void handle_keyboard_input();
void handle_controller_input();

bool active = false;

void setup(void) {
  Serial.begin(115200);
  delay(300);
  legs.init();
  delay(4000);
  imu.init();
  imu.calibrate();
  delay(500);
  imu.reset();
  controller.init();
}

void loop() {
  // Initialize a new data packet for data logging
  DataPacket packet;

  // Update IMU data
  imu.update();
  imu.print();

  // Handle inputs from keyboard
  handle_keyboard_input();
  handle_controller_input();

  // Run balance controller if applicable
  Pose pose = imu.get_data();
  if (!active && fabs(pose.pitch) < 0.1) // should be 0.1
    active = true;
  if (active && fabs(pose.pitch) < 20) {
    balance_control.balance(&pose, &packet);
    legs.balance_roll(&pose);
  }
  else {
    balance_control.reset();
    legs.scrunch();
    // active = false;
  }

  // Leg Testing  
  // legs.write_height(100,100);
  // delay(2000);
  // legs.write_height(0,0);
  // delay(2000);

  //delay to maintain loop integrity
  delay(2); //absolutely has to be 2ms update or PID breaks
}

void handle_controller_input() {
  float steering = controller.get_steering();
  steering = (fabs(steering) < 2)? 0: steering;
  float throttle = controller.get_throttle();
  throttle = (fabs(throttle) < 2)? 0: throttle;

  
  balance_control.set_turn_power(steering*0.3, -steering*0.3);
  balance_control.set_target_speed(-throttle*0.0012);
}

void handle_keyboard_input() {
  string serial_in = "";
  while (Serial.available()) {
    delay(3);
    char c = Serial.read();
    serial_in += c;
  }

  // WASD Keyboard Controls to drive
  if(serial_in == "w") {
    Serial.println("Forward!");
    balance_control.set_target_speed(4);
  }
  else if(serial_in == "s") {
    Serial.println("Backward!");
    balance_control.set_target_speed(-4);
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
  // K to raise the legs
  else if (serial_in == "k") {
    Serial.println("Raising!");
    legs.add_target_height(25);
    imu.adjust_offset(-0.5);
  }
  // J to lower the legs
  else if (serial_in == "j") {
    Serial.println("Lowering!");
    legs.add_target_height(-25);
    imu.adjust_offset(0.5);
  }
}