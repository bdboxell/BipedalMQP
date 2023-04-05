#include "pid.h"
#include "Arduino.h"

double pid_calculate(PID* pid, double target, double current) {
    double error = target - current;

    double delta_t = (millis() - pid->last_time)/1000.0;
    pid->sum += error*delta_t;
    
    if (!(pid->set)) {
        pid->last_error = error;
    }

    double pTerm = pid->kP*error;
    double iTerm = pid->kI*pid->sum;
    double dTerm = pid->kD*(error - pid->last_error)/delta_t;

    iTerm = (iTerm/fabs(iTerm))*((fabs(iTerm) > pid->max_i)? pid->max_i : fabs(iTerm));

    if (fabs(error)< pid->epsilon_inner || fabs(error) > pid->epsilon_outer) {
        // iTerm = 0;
        // dTerm = 0;
        pid->sum = pid->sum*0.9;
    }

    Serial.print(pTerm);
    Serial.print(",\t");
    Serial.print(iTerm);
    Serial.print(",\t");
    Serial.print(dTerm);

    double output = pTerm + iTerm + dTerm;
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