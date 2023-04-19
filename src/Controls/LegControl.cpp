#include "LegControl.h"

LegControl::LegControl() {
    target_height = 0;
    left_leg_PID = pid_init(8,0,0);
    right_leg_PID = pid_init(8,0,0);
}

// Initialization sets the height to reset position
void LegControl::init() {
    set_target_height(0);
    write_height(0,0);
}

// Controller for maintaining the target height and target roll
void LegControl::balance_roll(Pose* pose) {
    // Calculate the offset for each leg to maintain balance in roll
    // float roll_adjust = 210*tan((target_roll - pose->roll)*DEG_TO_RAD);
    float roll_adjust = 0;

    // Calculate the setpoint for each leg
    float left_target = target_height + roll_adjust;
    float right_target = target_height - roll_adjust;

    // PID calculations
    float left_velocity = pid_calculate(&left_leg_PID, left_target, last_left_pos);
    float right_velocity = pid_calculate(&right_leg_PID, right_target, last_right_pos);

    // Write the velocities to the motor
    write_velocity(left_velocity, right_velocity);
}

// A Function that writes a 'velocity' to the servo by continually calculating a new setpoint for the servo to go to.
void LegControl::write_velocity(float left, float right) {
    // Bound the desired velocities to be less than max
    left = bound(left, max_velocity);
    right = bound(right, max_velocity);

    // This check exists to assist the first loop iteration where the legs are enabled
    if (last_timestamp == 0) {
        last_timestamp = millis();
    }

    // Calculate the new setpoints
    float elapsed = (millis() - last_timestamp)/1000.0;
    last_left_pos += left*elapsed;
    last_right_pos += right*elapsed;

    // Write the new setpoints to servo
    write_height(last_left_pos, last_right_pos);

    // Update elapsed time
    last_timestamp = millis();
}

// Sets the height of each of the legs. Accepts range 0-100%
// 0 is folded all the way up, 100 is fully extended
void LegControl::write_height(float left, float right) {
    // Update the last motor positions since we're overriding velocity control
    last_left_pos = left;
    last_right_pos = right;

    // Convert 0 - 100% to -100 - 100% range
    left = (left*2)-100;
    right = (right*2)-100;

    // Send new values to servos
    left_motor.write_percent(left);
    right_motor.write_percent(right);
}

void LegControl::set_target_height(float height) {
    target_height = height;
}

void LegControl::add_target_height(double addend) {
    target_height+=addend;
    if (target_height > 100) target_height = 100;
    if (target_height < 0) target_height = 0;
}

// What to do when the robot falls over
void LegControl::scrunch() {
    write_height(0,0);
    last_left_pos = 0;
    last_right_pos = 0;
    last_timestamp = 0;
}