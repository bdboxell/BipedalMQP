#include "pid.h"
#include "Arduino.h"

double pid_calculate(PID *pid, double target, double current)
{
    // Calculate error and elapsed time
    double error = target - current;
    double delta_t = (millis() - pid->last_time) / 1000.0;

    // I term accumulator calculation
    pid->sum += error * delta_t;
    // Bound the I sum if it is out of a max range
    pid->sum = bound(pid->sum, pid->max_i);

    // Compensation to prevent strange values on first loop iteration
    if (!(pid->set))
    {
        pid->last_error = error;
        pid->set = true;
    }

    // D term calculation, uses a filtered change in error
    double delta_error = (error - pid->last_error) / delta_t;
    delta_error = pid->get_filtered_velocity(delta_error);

    // Calculate the P I and D terms
    pid->p_term = pid->kP * error;
    pid->i_term = pid->kI * pid->sum;
    pid->d_term = pid->kD * delta_error;

    // This is the criteria for when to reset the I term. The I accumulator will decay if the PID loop is within some tolerance of the target and the error is not rapidly changing. This indicates when the response has settled.
    if ((fabs(delta_error) < 0.3 && fabs(error) < pid->epsilon_inner) || fabs(error) > pid->epsilon_outer)
    {
        pid->sum = pid->sum * 0.95;
    }

    // Final output calculation
    pid->output = pid->p_term + pid->i_term + pid->d_term;

    // Update some variables
    pid->last_error = error;
    pid->last_time = millis();

    return pid->output;
}

void reset_pid(PID *pid)
{
    pid->last_error = 0;
    pid->sum = 0;
    pid->set = false;
    pid->set = millis();
}

void log_pid(PID *pid, DataPacket* packet) {
    packet->add_data("P", pid->kP);
    packet->add_data("I", pid->kI);
    packet->add_data("D", pid->kD);
    packet->add_data("Output", pid->output);
}

PID pid_init(double kP, double kI, double kD)
{
    PID pid;
    pid.kP = kP;
    pid.kI = kI;
    pid.kD = kD;
    pid.last_time = millis();
    return pid;
}
PID pid_init(double kP, double kI, double kD, double epsilon_inner, double epsilon_outer, double max_i)
{
    PID pid;
    pid.kP = kP;
    pid.kI = kI;
    pid.kD = kD;
    pid.epsilon_inner = epsilon_inner;
    pid.epsilon_outer = epsilon_outer;
    pid.max_i = max_i;
    pid.last_time = millis();
    return pid;
}

void pid_init(PID* pid, double kP, double kI, double kD, double epsilon_inner, double epsilon_outer, double max_i)
{
    pid->kP = kP;
    pid->kI = kI;
    pid->kD = kD;
    pid->epsilon_inner = epsilon_inner;
    pid->epsilon_outer = epsilon_outer;
    pid->max_i = max_i;
    pid->last_time = millis();
}