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
    // imu.configSPI();
    burstData = imu.wordBurst(); // Read data and insert into array
    // Serial.println(imu.regRead(0x04));
}

void IMU::init() {
    IMU_obj = this;
    imu.resetDUT(10);
    imu.configSPI();
    delay(500);
    imu.regWrite(MSC_CTRL, 0xC1);
    imu.regWrite(FILT_CTRL, 0x04); // Set digital filter
    imu.regWrite(DEC_RATE, 0x00), // Disable decimation

    // Read the control registers once to print to screen
    // MSC = imu.regRead(MSC_CTRL);
    // FLTR = imu.regRead(FILT_CTRL);
    // DECR = imu.regRead(DEC_RATE);
    
    attachInterrupt(dr_pin, ISR, RISING);
}

// Called on every rising edge of Data Ready Pin
// Handles all IMU data updating in the background
void IMU::ISR() {
    // Serial.println("ISR");
    int delta_t = micros() - ((IMU*) IMU_obj)->lastTimeStamp;
    if (delta_t > 500) {
        ((IMU*) IMU_obj)->grabData();
        ((IMU*) IMU_obj)->scaleData();
        // Pose data = ((IMU*) IMU_obj)->get_data();
        ((IMU*) IMU_obj)->updateData();
    }
}

void IMU::updateData() {
    // get_data(&raw_data);
    Pose norm_data;
    norm_data.pitch = raw_data.pitch;
    norm_data.roll = raw_data.roll;
    norm_data.yaw = raw_data.yaw;
    norm_data.x = raw_data.x;
    norm_data.y = raw_data.y;
    norm_data.z = raw_data.z;

    normalize_data(&norm_data);
    double elapsed = (micros() - lastTimeStamp)/1000000.0;
    lastTimeStamp = micros();
    // Pose cur_pose;

    cur_pose.x = norm_data.x;
    cur_pose.y = norm_data.y;
    cur_pose.z = norm_data.z;

    float accel_roll = -atan(norm_data.x/sqrt(norm_data.y*norm_data.y+norm_data.z*norm_data.z))*1/(3.142/180);
    float accel_pitch = atan(norm_data.y/sqrt(norm_data.x*norm_data.x+norm_data.z*norm_data.z))*1/(3.142/180);

    kalman_filter(elapsed, norm_data.pitch, accel_pitch, pitch_filter);
    kalman_filter(elapsed, norm_data.roll, accel_roll, roll_filter);
    // Serial.println(accel_roll);

    // cur_pose.pitch += norm_data.pitch*elapsed;
    // cur_pose.roll += norm_data.roll*elapsed;
    cur_pose.pitch = pitch_filter[0];
    cur_pose.roll = roll_filter[0];
    cur_pose.yaw += norm_data.yaw*elapsed;
}

void IMU::normalize_data(Pose* data) {
    data->roll -= average_filter.roll;
    data->pitch -= average_filter.pitch;
    data->yaw -= average_filter.yaw;
    
    data->x -= 0.15;
    data->y -= 0.165;
    data->z -= 0.18;
}


// Function used to scale all acquired data (scaling functions are included in ADIS16470.cpp)
void IMU::scaleData()
{
    raw_data.pitch = imu.gyroScale(*(burstData + 1)); //Scale X Gyro
    raw_data.roll = imu.gyroScale(*(burstData + 2)); //Scale Y Gyro
    raw_data.yaw = imu.gyroScale(*(burstData + 3)); //Scale Z Gyro
    raw_data.x = imu.accelScale(*(burstData + 4)); //Scale X Accel
    raw_data.y = imu.accelScale(*(burstData + 5)); //Scale Y Accel
    raw_data.z = imu.accelScale(*(burstData + 6)); //Scale Z Accel
    TEMPS = imu.tempScale(*(burstData + 7)); //Scale Temp Sensor
}

void IMU::calibrate() {
    Serial.println("Calibrating IMU...");
    for (int i = 0; i < 300; ++i) {
        delay(10);
        average_filter.x = (average_filter.x*i + raw_data.x)/(i+1);
        average_filter.y = (average_filter.y*i + raw_data.y)/(i+1);
        average_filter.z = (average_filter.z*i + raw_data.z)/(i+1);
        average_filter.roll = (average_filter.roll*i + raw_data.roll)/(i+1);
        average_filter.pitch = (average_filter.pitch*i + raw_data.pitch)/(i+1);
        average_filter.yaw = (average_filter.yaw*i + raw_data.yaw)/(i+1);
    }
    Serial.println("Calibration Successful!");
}

void IMU::calibrate_pitch() {

}

// Uses pass by reference to fill a pose object with IMU data
void IMU::get_data(Pose* pose) {
    pose->pitch = cur_pose.pitch + pitch_offset;
    pose->roll = cur_pose.roll;
    pose->yaw = cur_pose.yaw;
    pose->x = cur_pose.x;
    pose->y = cur_pose.y;
    pose->z = cur_pose.z;
}

// Returns a copy of the IMU data
Pose IMU::get_data() {
    Pose pose;
    get_data(&pose);
    // Serial.println(MOSI);
    return pose;
}

void IMU::kalman_filter(double elapsed_s, double k_input, double k_measure, double* output) {
    double gyro_uncertain = 3;
    double acc_uncertain = 1;
    double k_state = output[0] + elapsed_s*k_input;
    double k_uncertainty = output[1] + elapsed_s * elapsed_s * gyro_uncertain*gyro_uncertain;
    double k_gain = k_uncertainty * 1/(k_uncertainty + acc_uncertain*acc_uncertain);
    k_state = k_state + k_gain*(k_measure - k_state);
    k_uncertainty = (1-k_gain) * k_uncertainty;
    output[0] = k_state;
    output[1] = k_uncertainty;
}

void IMU::reset() {
    cur_pose.pitch = 0;
    cur_pose.roll = 0;
    cur_pose.yaw = 0;
}