#include "pid.h"
#include "Arduino.h"

float pid_calculate(PID* pid, float target, float current) {
    float error = target - current;

    float delta_t = (millis() - pid->last_time)/1000.0;
    pid->sum += error*delta_t;
    
    if (!(pid->set)) {
        pid->last_error = error;
    }

    float pTerm = pid->kP*error;
    float iTerm = pid->kI*pid->sum;
    float dTerm = pid->kD*(error - pid->last_error)/delta_t;

    iTerm = (iTerm/fabs(iTerm))*((fabs(iTerm) > pid->max_i)? pid->max_i : fabs(iTerm));

    if (fabs(error)< pid->epsilon_inner || fabs(error) > pid->epsilon_outer) {
        iTerm = 0;
        // dTerm = 0;
        pid->sum = 0;
    }

    Serial.print(pTerm);
    Serial.print(",\t");
    Serial.print(iTerm);
    Serial.print(",\t");
    Serial.print(dTerm);

    float output = pTerm + iTerm + dTerm;
    Serial.print(",\t");
    Serial.println(output);
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

PID pid_init(float kP, float kI, float kD) {
    PID pid;
    pid.kP = kP;
    pid.kI = kI;
    pid.kD = kD;
    pid.last_time = millis();
    return pid;
}
PID pid_init(float kP, float kI, float kD, float epsilon_inner, float epsilon_outer, float max_i) {
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