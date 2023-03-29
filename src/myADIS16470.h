#include <SPI.h>
#include <Arduino.h>

class myADIS16470 {
    public:
        // Constructor
        myADIS16470(int CS, int DR, int RST);
        // Destructor
        ~myADIS16470();
        // Hardware Reset
        int reset();
        // Select
        int select();
        // Deselect
        int deslect();
        // Reads sensor data in a burst
        uint8_t* byteBurst();
        // Configures SPI parameters
        void configSPI();
    private:
        int _CS;
        int _DR;
        int _RST;
        int stall = 20;

        int hardware_reset_time = 10; //ms it takes for ADIS16470 to hardware reset
};