#include <IMU.h>

void* IMU::IMU_obj = nullptr;

IMU::IMU() {
}

IMU::~IMU() {
    // IMU_obj = nullptr;
}

void IMU::grabData()
{
    burstData = {};
    imu.configSPI();
    burstData = imu.wordBurst(); // Read data and insert into array
}

void IMU::init() {
    IMU_obj = this;
    imu.configSPI();
    delay(500);
    // imu.regWrite(MSC_CTRL, 0xC1);
    // imu.regWrite(FILT_CTRL, 0x04); // Set digital filter
    // imu.regWrite(DEC_RATE, 0x00), // Disable decimation

    // Read the control registers once to print to screen
    // MSC = imu.regRead(MSC_CTRL);
    // FLTR = imu.regRead(FILT_CTRL);
    // DECR = imu.regRead(DEC_RATE);
    
    attachInterrupt(dr_pin, ISR, RISING);
}

void IMU::ISR() {
    // Serial.println("ISR");
    ((IMU*) IMU_obj)->grabData();
    ((IMU*) IMU_obj)->scaleData();
}

// Function used to scale all acquired data (scaling functions are included in ADIS16470.cpp)
void IMU::scaleData()
{
    GXS = imu.gyroScale(*(burstData + 1)); //Scale X Gyro
    GYS = imu.gyroScale(*(burstData + 2)); //Scale Y Gyro
    GZS = imu.gyroScale(*(burstData + 3)); //Scale Z Gyro
    AXS = imu.accelScale(*(burstData + 4)); //Scale X Accel
    AYS = imu.accelScale(*(burstData + 5)); //Scale Y Accel
    AZS = imu.accelScale(*(burstData + 6)); //Scale Z Accel
    TEMPS = imu.tempScale(*(burstData + 7)); //Scale Temp Sensor
    // Serial.println(GXS);
}

void IMU::calibrate() {

}

void IMU::calibrate_pitch() {

}

// Uses pass by reference to fill a pose object with IMU data
void IMU::get_data(Pose* pose) {
    pose->pitch = GXS;
    pose->roll = GYS;
    pose->yaw = GXS;
    pose->x = AXS;
    pose->y = AYS;
    pose->z = AZS;
}

// Returns a copy of the IMU data
Pose IMU::get_data() {
    Pose pose;
    get_data(&pose);
    // Serial.println(MOSI);
    return pose;
}