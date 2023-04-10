struct PID {
    double kP, kI, kD;
    double epsilon_inner = 0;
    double epsilon_outer = 99999;
    double last_error = 0;
    double max_i = 999999;
    double sum = 0;
    bool set = false;
    unsigned long last_time;
    double velocity_filter[5] = {0,0,0,0,0};

    double get_filtered_velocity(double new_val) {
        double sum = 0;
        for (int i = 0 ; i< 4; i++) {
            velocity_filter[i] = velocity_filter[i+1];
            sum+=velocity_filter[i];
        }
        sum+=new_val;
        velocity_filter[4] = new_val;
        return sum/5;
    }
};

double pid_calculate(PID* pid, double target, double current);
void reset_pid(PID* pid);
PID pid_init(double kP, double kI, double kD);
PID pid_init(double kP, double kI, double kD, double epsilon_inner, double epsilon_outer, double max_i);