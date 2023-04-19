#include "../Actuators/MotorInterface.h"
#include "../Controls/pid.h"
#include "../Utilities/MathUtils.h"

#define left_servo_pin 14
#define right_servo_pin 27

class LegControl {
    public:
        LegControl();

        void init();
        void write_height(float left, float right);
        void balance_roll(Pose* pose);
        void set_target_height(float height);
        void add_target_height(double addend);
        void write_velocity(float left, float right);
        void scrunch();
    private:
        double target_height = 0;
        float target_roll = 0;

        const float max_velocity = 80;
        float last_left_pos = 0;
        float last_right_pos = 0;
        unsigned long last_timestamp = 0;
        MotorInterface right_motor = MotorInterface(right_servo_pin, false, 700,1800); //left is the retracted, right is extended
        MotorInterface left_motor = MotorInterface(left_servo_pin, true, 1000,2100); //left is extended, right is retract
        PID left_leg_PID;
        PID right_leg_PID;
};