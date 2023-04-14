#include "Balance.h"

Balance::Balance()
{
    init();
}

void Balance::init()
{
    // PID balance_pid = pid_init(45, 800, 3.2, 0.015, 0.75, 99999); //This was used in the youtube short
    // PID balance_pid = pid_init(120, 875, 0.625, 0.015, 0.75, 99999); //P Heavy Testing , Ku = 125
    // balance_pid = pid_init(45, 1000, 3.8, 0.015, 0.75, 99999); // Demo'd to Agheli
    balance_pid = pid_init(45, 890, 5, 0.015, 0.75, 99999); // Carpet Testing

    angle_pid = pid_init(1.0,1.0,1.0,0,99999,99999);
}

void Balance::balance(Pose *pose, DataPacket* packet)
{
    // For all calculations, the speed is the average power sent to the motors over the past <filter_size> iterations

    // Calculate what the target angle should be.
    // double target_angle = pid_calculate(&angle_pid, 5, 0);
    // Serial.println(angle_pid.kP);
    double target_angle = 0;

    // The control variable for the balance PID is the projection of the COM on the horizontal axis
    double com_displace = 4.5 * sin(pose->pitch * DEG_TO_RAD);
    float power = pid_calculate(&balance_pid, target_angle, com_displace) - 2000 * sin(target_angle * DEG_TO_RAD);

    // Update the speed filter
    filter_speed(power);

    // Adjust left and right powers to handle turning
    float left_power = power + left_input;
    float right_power = power + right_input;

    // Make sure that neither power value is over the maximum
    left_power = bound(left_power, max_speed);
    right_power = bound(right_power, max_speed);

    // Write powers to the drive motors
    right_motor.write_percent(right_power);
    left_motor.write_percent(left_power);

    // Handle real-time data logging
    if (logging) {
        packet->add_data("pitch", pose->pitch);
        packet->add_data("power", power);
        packet->add_data("target_angle", target_angle);
        packet->add_data("speed", cur_speed);
        log_pid(&angle_pid, packet);

        // Limit the data logging to 5 iterations = 10ms refresh rate
        if (iter_count > 5)
        {
            packet->write_to_serial();
            iter_count = 0;
        }
    }
    iter_count++;
}

void Balance::filter_speed(double new_val)
{
    velocity_sum -= velocity_filter[velocity_filter_index];
    velocity_filter[velocity_filter_index] = new_val;
    velocity_sum += new_val;
    velocity_filter_index ++;
    if (velocity_filter_index == filter_size) {
        velocity_filter_index = 0;
    }
    cur_speed = velocity_sum / filter_size;
}

void Balance::start_logging()
{
    logging = true;
}

void Balance::stop_logging()
{
    logging = false;
}

void Balance::set_target_speed(double speed)
{
    target_speed = speed;
}

void Balance::add_turn_power(float left, float right)
{
    left_input += left;
    right_input += right;
}

void Balance::stop()
{
    left_input = 0;
    right_input = 0;
    target_speed = 0;
}

void Balance::toggle_logging()
{
    logging = !logging;
}

void Balance::reset()
{
    reset_pid(&balance_pid);
    left_motor.stop();
    right_motor.stop();
}