struct PID {
    float kP, kI, kD;
    float epsilon_inner = 0;
    float epsilon_outer = 99999;
    float last_error = 0;
    float sum = 0;
    bool set = false;
    unsigned long last_time;
};

float pid_calculate(PID* pid, float target, float current);
void reset_pid(PID* pid);
PID pid_init(float kP, float kI, float kD);
PID pid_init(float kP, float kI, float kD, float epsilon_inner, float epsilon_outer);