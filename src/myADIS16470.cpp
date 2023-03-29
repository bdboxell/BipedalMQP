#include "myADIS16470.h"

myADIS16470::myADIS16470(int CS, int DR, int RST): _CS{CS}, _DR{DR}, _RST{RST}
{
    //Initialize pins besides the standard SPI pins
    pinMode(_CS, OUTPUT);
    pinMode(_DR, INPUT);
    pinMode(_RST, OUTPUT);

    digitalWrite(_CS, HIGH);
    digitalWrite(_RST, HIGH);

    configSPI();
}

// Activates CS to begin SPI transaction
int myADIS16470::select() {
    digitalWrite(_CS, LOW);
    return 1;
}

// Performs a hardware reset on the ADIS16470
int myADIS16470::reset() {
    digitalWrite(_RST, LOW);
    delay(hardware_reset_time);
    digitalWrite(_RST, HIGH);
    delay(hardware_reset_time);
    return 1;
}

void myADIS16470::configSPI() {
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV64);
    SPI.setDataMode(SPI_MODE3);
    SPI.begin();
}
