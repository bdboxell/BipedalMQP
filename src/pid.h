struct PID {
    double kP, kI, kD;
    double epsilon_inner = 0;
    double epsilon_outer = 99999;
    double last_error = 0;
    double max_i = 999999;
    double sum = 0;
    bool set = false;
    unsigned long last_time;
};

double pid_calculate(PID* pid, double target, double current);
void reset_pid(PID* pid);
PID pid_init(double kP, double kI, double kD);
PID pid_init(double kP, double kI, double kD, double epsilon_inner, double epsilon_outer, double max_i);