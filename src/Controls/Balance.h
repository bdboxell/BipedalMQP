#include "pid.h"
#include "../Actuators/MotorInterface.h"
#include "../Utilities/MathUtils.h"
#include "../Utilities/DataPacket.h"

class Balance {
    public:
        Balance();
        void balance(Pose* pose, DataPacket* packet);
        void init();
        void start_logging();
        void stop_logging();
        void toggle_logging();
        void set_target_speed(double speed);
        void add_turn_power(float left, float right);
        void stop();
        void reset();

    private:
        PID balance_pid;
        PID angle_pid;

        MotorInterface right_motor = MotorInterface(14, false);
        MotorInterface left_motor = MotorInterface(12, true);

        const static int filter_size = 500;
        double velocity_sum = 0;
        int velocity_filter_index = 0;
        double velocity_filter[filter_size];
        void filter_speed(double new_val);

        bool logging = false;

        float max_speed = 100;
        double target_speed = 0;

        int iter_count = 0;

        float left_input = 0;
        float right_input = 0;

        float cur_speed = 0;
};