#include "pid.h"
#include "Arduino.h"

float pid_calculate(PID* pid, float target, float current) {
    float error = target - current;
    pid->sum += error;
    if (!(pid->set)) {
        pid->last_error = error;
    }

    float pTerm = pid->kP*error;
    float iTerm = pid->kI*pid->sum;
    float dTerm = pid->kD*(error - pid->last_error);

    if (fabs(current)< pid->epsilon_inner || fabs(current) > pid->epsilon_outer) {
        iTerm = 0;
        pid->sum = 0;
    }

    float output = pTerm + iTerm + dTerm;
    pid->last_error = error;
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
}

PID pid_init(float kP, float kI, float kD) {
    PID pid;
    pid.kP = kP;
    pid.kI = kI;
    pid.kD = kD;
    return pid;
}
PID pid_init(float kP, float kI, float kD, float epsilon_inner, float epsilon_outer) {
    PID pid;
    pid.kP = kP;
    pid.kI = kI;
    pid.kD = kD;
    pid.epsilon_inner = epsilon_inner;
    pid.epsilon_outer = epsilon_outer;
    return pid;
}