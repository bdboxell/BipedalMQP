#include "MPU6050.h"
#include <vector>

using namespace std;

MPU6050::MPU6050() {

}

void MPU6050::init() {
     while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void MPU6050::get_raw_data(Pose* pose) {
     /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    pose->x = a.acceleration.y;
    pose->y = a.acceleration.z;
    pose->z = a.acceleration.x;
    pose->pitch = g.gyro.y * RAD_TO_DEG;
    pose->roll = g.gyro.z * RAD_TO_DEG;
    pose->yaw = g.gyro.x * RAD_TO_DEG;
}

void MPU6050::update() {
    Pose raw_data;
    get_raw_data(&raw_data);
    normalize_data(&raw_data);
    float elapsed = (millis() - lastTimeStamp)/1000.0;
    lastTimeStamp = millis();

    cur_pose.pitch += raw_data.pitch*elapsed;
    cur_pose.roll += raw_data.roll*elapsed;
    cur_pose.yaw += raw_data.yaw*elapsed;

    cur_pose.x = raw_data.x;
    cur_pose.y = raw_data.y;
    cur_pose.z = raw_data.z;
}

void MPU6050::normalize_data(Pose* data) {
    // data->x -= average_filter.x;
    // data->y -= average_filter.y;
    // data->z -= average_filter.z;
    data->roll -= average_filter.roll;
    data->pitch -= average_filter.pitch;
    data->yaw -= average_filter.yaw;
}

void MPU6050::calibrate() {
    Serial.println("Calibrating IMU...");
    for (int i = 0; i < 300; ++i) {
        delay(10);
        Pose cur_raw_data;
        get_raw_data(&cur_raw_data);
        average_filter.x = (average_filter.x*i + cur_raw_data.x)/(i+1);
        average_filter.y = (average_filter.y*i + cur_raw_data.y)/(i+1);
        average_filter.z = (average_filter.z*i + cur_raw_data.z)/(i+1);
        average_filter.roll = (average_filter.roll*i + cur_raw_data.roll)/(i+1);
        average_filter.pitch = (average_filter.pitch*i + cur_raw_data.pitch)/(i+1);
        average_filter.yaw = (average_filter.yaw*i + cur_raw_data.yaw)/(i+1);
    }
    Serial.println("Calibration Successful!");
}

void MPU6050::calibrate_pitch() {
  Pose raw_data;
  get_raw_data(&raw_data);
  float theta1 = acos(raw_data.z/g);
  // float theta2 = acos(x/g);
  // float offset = theta1 *RAD_TO_DEG;
  float offset = -78.75;
  cur_pose.pitch += offset;//offset+fudge_factor;
}

void MPU6050::get_data(Pose* pose) {
    pose->x = cur_pose.x;
    pose->y = cur_pose.y;
    pose->z = cur_pose.z;
    pose->roll = cur_pose.roll;
    pose->pitch = cur_pose.pitch;
    pose->yaw = cur_pose.yaw;
}

Pose MPU6050::get_data() {
    return cur_pose;
}

void MPU6050::print() {
    // Serial.print(" Roll = ");
    // Serial.print(cur_pose.roll);
    Serial.print("\nPitch = ");
    Serial.print(cur_pose.pitch);
    // Serial.print(" Yaw = ");
    // Serial.println(cur_pose.yaw);
    
    // Serial.print(" X = ");
    // Serial.print(cur_pose.x);
    // Serial.print(" Y = ");
    // Serial.print(cur_pose.y);
    // Serial.print(" Z = ");
    // Serial.println(cur_pose.z);
}