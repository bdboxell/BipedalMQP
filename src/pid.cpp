#include "pid.h"
#include "Arduino.h"

double pid_calculate(PID* pid, double target, double current) {
    double error = target - current;

    double delta_t = (millis() - pid->last_time)/1000.0;
    pid->sum += error*delta_t;
    
    if (!(pid->set)) {
        pid->last_error = error;
        pid->set = true;
    }

    double delta_error = (error - pid->last_error)/delta_t;
    delta_error = pid->get_filtered_velocity(delta_error);

    double pTerm = pid->kP*error;
    double iTerm = pid->kI*pid->sum;
    double dTerm = pid->kD*delta_error;

    iTerm = (iTerm/fabs(iTerm))*((fabs(iTerm) > pid->max_i)? pid->max_i : fabs(iTerm));
    // Serial.println(pid->epsilon_inner);
    // I stutters a lot when it passes this band. Maybe add another qualifier to the if statement for if the rate of change of the error is below a threshhold
    if ((fabs(delta_error) < 0.2 && fabs(error) < pid->epsilon_inner) || fabs(error) > pid->epsilon_outer) {
        // iTerm = 0;
        // dTerm = 0;
        pid->sum = pid->sum*0.95;
        // Serial.println("Resetting I!");
    }

    double output = pTerm + iTerm + dTerm;
    // Serial.print(pTerm);
    // Serial.print(",\t");
    // Serial.print(iTerm);
    // Serial.print(",\t");
    // Serial.print(dTerm);
    // Serial.print(",\t");
    // Serial.println(output);
    pid->last_error = error;
    pid->last_time = millis();
    // Serial.print("kP: ");
    // Serial.print(pid->kP);
    // Serial.print(" Error: ");
    // Serial.print(error);
    // Serial.print(" Output: ");
    // Serial.println(output);
    return output;
}

void reset_pid(PID* pid) {
    pid->last_error = 0;
    pid->sum = 0;
    pid->set = false;
    pid->set = millis();
}

PID pid_init(double kP, double kI, double kD) {
    PID pid;
    pid.kP = kP;
    pid.kI = kI;
    pid.kD = kD;
    pid.last_time = millis();
    return pid;
}
PID pid_init(double kP, double kI, double kD, double epsilon_inner, double epsilon_outer, double max_i) {
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