#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <Adafruit_Sensor.h>

#include "motor.h"
#include "encoder.h"

#include "config.h"

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

// #include <Arduino-ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BusIO_Register.h>

ros::NodeHandle nh;

/////////////PID///////////////
#include <PID_v1.h>
// Define Variables we'll be connecting to

double Setpoint1, Input1, Output1, Output11;
double Setpoint2, Input2, Output2, Output22;
double Setpoint3, Input3, Output3, Output33;
double Setpoint4, Input4, Output4, Output44;

// Specify the links and initial tuning parameters
double Kp1 = 1.32, Ki1 = 0, Kd1 = 0.012;
double Kp2 = 1.32, Ki2 = 0, Kd2 = 0.012;
double Kp3 = 1.32, Ki3 = 0, Kd3 = 0.012;
double Kp4 = 1.32, Ki4 = 0, Kd4 = 0.012;

PID motor1_PID(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID motor2_PID(&Input2, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);
PID motor3_PID(&Input3, &Output3, &Setpoint3, Kp3, Ki3, Kd3, DIRECT);
PID motor4_PID(&Input4, &Output4, &Setpoint4, Kp4, Ki4, Kd4, DIRECT);

/////////////// MOTOR ///////////////
Motor motor1(MOTOR1A, MOTOR1B);
Motor motor2(MOTOR2A, MOTOR2B);
Motor motor3(MOTOR3A, MOTOR3B);
Motor motor4(MOTOR4A, MOTOR4B);

void motor1_callback(const std_msgs::Int16 &speed);
void motor2_callback(const std_msgs::Int16 &speed);
void motor3_callback(const std_msgs::Int16 &speed);
void motor4_callback(const std_msgs::Int16 &speed);

ros::Subscriber<std_msgs::Int16> sub1("motor1", &motor1_callback);
ros::Subscriber<std_msgs::Int16> sub2("motor2", &motor2_callback);
ros::Subscriber<std_msgs::Int16> sub3("motor3", &motor3_callback);
ros::Subscriber<std_msgs::Int16> sub4("motor4", &motor4_callback);

////////////// ENCODER //////////////
Encoder encoder1(ENC1A, ENC1B, CPR, false);
Encoder encoder2(ENC2A, ENC2B, CPR, false);
Encoder encoder3(ENC3A, ENC3B, CPR, false);
Encoder encoder4(ENC4A, ENC4B, CPR, false);

std_msgs::Float32 enc1;
std_msgs::Float32 enc2;
std_msgs::Float32 enc3;
std_msgs::Float32 enc4;

ros::Publisher pub1("enc1", &enc1);
ros::Publisher pub2("enc2", &enc2);
ros::Publisher pub3("enc3", &enc3);
ros::Publisher pub4("enc4", &enc4);

// geometry_msgs::Pose pose;
// ros::Publisher pub6("mwmr_pose", &pose);

////////// IMU ///////////
Adafruit_ICM20948 icm;

sensor_msgs::Imu imu_data;
std_msgs::Float32 heading_data;
ros::Publisher pub5("/imu/data", &imu_data);
ros::Publisher pub6("/heading", &heading_data);

// declare the function protoype
void encoder1Update();
void encoder2Update();
void encoder3Update();
void encoder4Update();
// void send_pose_data();

void send_imu_data();

// void PIDcallback();
// void computePID();

void send_encoder_data();
unsigned long prev_time;

void setup()
{
  nh.initNode();
  // motor
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  // encoder
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);
  nh.advertise(pub4);

  // imu
  nh.advertise(pub5);
  nh.advertise(pub6);

  // PIDcallback();

  delay(100);

  attachInterrupt(digitalPinToInterrupt(ENC1A), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1B), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), encoder2Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), encoder2Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3A), encoder3Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3B), encoder3Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4A), encoder4Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4B), encoder4Update, CHANGE);

  prev_time = millis();

  // Setpoint1=0;

  // turn the PID on
  motor1_PID.SetOutputLimits(-255, 255);
  motor2_PID.SetOutputLimits(-255, 255);
  motor3_PID.SetOutputLimits(-255, 255);
  motor4_PID.SetOutputLimits(-255, 255);

  motor1_PID.SetMode(AUTOMATIC);
  motor2_PID.SetMode(AUTOMATIC);
  motor3_PID.SetMode(AUTOMATIC);
  motor4_PID.SetMode(AUTOMATIC);

  while(!icm.begin_I2C())
  {
    delay(1000);
  }
}

void loop()
{
  nh.spinOnce();
  if (millis() - prev_time > ENC_DELAY)
  {
    send_encoder_data();
    prev_time = millis();
  }
  delay(20);

  // Input=encoder1.get_rpm();
  // Setpoint=speed.data;
  // motor1_PID.Compute();

  // analogWrite(MOTOR1A, Output);

  Input1 = enc1.data;
  motor1_PID.Compute();
  motor1.setSpeed(Output1);

  Input2 = enc2.data;
  motor2_PID.Compute();
  motor2.setSpeed(Output2);

  Input3 = enc3.data;
  motor3_PID.Compute();
  motor3.setSpeed(Output3);

  Input4 = enc4.data;
  motor4_PID.Compute();
  motor4.setSpeed(Output4);

 

  send_imu_data();
}

void send_encoder_data()
{
  enc1.data = encoder1.get_rpm();
  enc2.data = encoder2.get_rpm();
  enc3.data = encoder3.get_rpm();
  enc4.data = encoder4.get_rpm();
  pub1.publish(&enc1);
  pub2.publish(&enc2);
  pub3.publish(&enc3);
  pub4.publish(&enc4);
}

void motor1_callback(const std_msgs::Int16 &speed)
{
  Setpoint1 = speed.data;
}

void motor2_callback(const std_msgs::Int16 &speed)
{
  Setpoint2 = speed.data;
}

void motor3_callback(const std_msgs::Int16 &speed)
{
  Setpoint3 = speed.data;
}

void motor4_callback(const std_msgs::Int16 &speed)
{

  Setpoint4 = speed.data;
}

void encoder1Update()
{
  encoder1.encoderUpdate();
}

void encoder2Update()
{
  encoder2.encoderUpdate();
}

void encoder3Update()
{
  encoder3.encoderUpdate();
}

void encoder4Update()
{
  encoder4.encoderUpdate();
}

void send_imu_data()
{
  sensors_event_t accel, gyro, mag, temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);
  heading_data.data = atan2(mag.magnetic.y, mag.magnetic.x) * 180 / 3.142;

  // imu::Vector<3> acc;
  // acc.x() = accel.acceleration.x;
  // acc.y() = accel.acceleration.y;
  // acc.z() = accel.acceleration.z;

  // imu::Quaternion quat;
  // quat.w() = 1.0; // Default value for the quaternion's scalar component
  // quat.x() = gyro.gyro.x;
  // quat.y() = gyro.gyro.y;
  // quat.z() = gyro.gyro.z;

  // imu_data.linear_acceleration.x = acc.x();
  // imu_data.linear_acceleration.y = acc.y();
  // imu_data.linear_acceleration.z = acc.z();
  // imu_data.orientation.w = quat.w();
  // imu_data.orientation.x = quat.x();
  // imu_data.orientation.y = quat.y();
  // imu_data.orientation.z = quat.z();

  // Calculate quaternion
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float gx = gyro.gyro.x;
  float gy = gyro.gyro.y;
  float gz = gyro.gyro.z;
  float qw = sqrt(1 + ax * ax + ay * ay + az * az) / 2.0;
  float qx = (gy * az - gz * ay) / (4 * qw);
  float qy = (gz * ax - gx * az) / (4 * qw);
  float qz = (gx * ay - gy * ax) / (4 * qw);

  // Publish quaternion
  imu_data.orientation.w = qw;
  imu_data.orientation.x = qx;
  imu_data.orientation.y = qy;
  imu_data.orientation.z = qz;

  // Publish linear acceleration
  imu_data.linear_acceleration.x = accel.acceleration.x;
  imu_data.linear_acceleration.y = accel.acceleration.y;
  imu_data.linear_acceleration.z = accel.acceleration.z;

  // icm20948.task();
  // float quat_w, quat_x, quat_y, quat_z;
  // if (icm20948.quat6DataIsReady())
  // {
  //   icm20948.readQuat6Data(&quat_w, &quat_x, &quat_y, &quat_z);
  // }

  // // Read acceleration data
  // float accel_x, accel_y, accel_z;
  // if (icm20948.accelDataIsReady())
  // {
  //   icm20948.readAccelData(&accel_x, &accel_y, &accel_z);
  // }
  // imu_data.linear_acceleration.x = accel_x;
  // imu_data.linear_acceleration.y = accel_y;
  // imu_data.linear_acceleration.z = accel_z;
  // imu_data.orientation.w = quat_w;
  // imu_data.orientation.x = quat_x;
  // imu_data.orientation.y = quat_y;
  // imu_data.orientation.z = quat_z;

  pub5.publish(&imu_data);
  pub6.publish(&heading_data);
}

// #include <Arduino.h>
// #include <math.h>
// #include <Adafruit_ICM20X.h>
// #include <Adafruit_ICM20948.h>
// Adafruit_ICM20948 icm;
// sensors_event_t accel, gyro, mag, temp;
// void setup()
// {
//   Serial.begin(115200);
//   // Try to initialize!
//   if (!icm.begin_I2C())
//   {
//     // if (!icm.begin_SPI(ICM_CS)) {
//     // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

//     Serial.println("Failed to find ICM20948 chip");
//     while (1)
//     {
//       delay(10);
//     }
//     // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
//     Serial.print("Magnetometer data rate set to: ");
//     switch (icm.getMagDataRate())
//     {
//     case AK09916_MAG_DATARATE_SHUTDOWN:
//       Serial.println("Shutdown");
//       break;
//     case AK09916_MAG_DATARATE_SINGLE:
//       Serial.println("Single/One shot");
//       break;
//     case AK09916_MAG_DATARATE_10_HZ:
//       Serial.println("10 Hz");
//       break;
//     case AK09916_MAG_DATARATE_20_HZ:
//       Serial.println("20 Hz");
//       break;
//     case AK09916_MAG_DATARATE_50_HZ:
//       Serial.println("50 Hz");
//       break;
//     case AK09916_MAG_DATARATE_100_HZ:
//       Serial.println("100 Hz");
//       break;
//     }
//     Serial.println();
//   }
// }

// void loop()
// {
//   icm.getEvent(&accel, &gyro, &temp, &mag);
//   Serial.print(atan2(mag.magnetic.y, mag.magnetic.x) * 180 / 3.142);
//   Serial.print("\t\tMag X: ");
//   Serial.print(mag.magnetic.x);
//   Serial.print(" \tY: ");
//   Serial.print(mag.magnetic.y);
//   Serial.print(" \tZ: ");
//   Serial.print(mag.magnetic.z);
//   Serial.println(" uT");
// }