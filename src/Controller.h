class Controller {
    private:
        const float ms_min = 1000;
        const float ms_max = 2000;
        int throttle_pin;
        int steering_pin;

        inline float scale(int input);
    public:
        Controller(int t_pin, int s_pin);
        float get_throttle();
        float get_steering();
};