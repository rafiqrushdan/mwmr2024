#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include "motor.h"
#include "encoder.h"

#include "config.h"

ros::NodeHandle nh;

  /////////////PID///////////////
  #include <PID_v1.h>
  //Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Setpoint1, Input1, Output1;

//Specify the links and initial tuning parameters
double Kp1=2, Ki1=0, Kd1=0;
double Kp2=2, Ki2=5, Kd2=1;
double Kp3=32, Ki3=5, Kd3=1;
double Kp4=2, Ki4=5, Kd4=1;

PID motor1_PID(&Input1, &Output1, &Setpoint1, Kp1, Ki1, Kd1, DIRECT);
PID motor2_PID(&Input, &Output, &Setpoint, Kp2, Ki2, Kd2, DIRECT);
PID motor3_PID(&Input, &Output, &Setpoint, Kp3, Ki3, Kd3, DIRECT);
PID motor4_PID(&Input, &Output, &Setpoint, Kp4, Ki4, Kd4, DIRECT);





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

void encoder1Update();
void encoder2Update();
void encoder3Update();
void encoder4Update();

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

Setpoint=0;

  //turn the PID on
  motor1_PID.SetMode(AUTOMATIC);
  motor2_PID.SetMode(AUTOMATIC);
  motor3_PID.SetMode(AUTOMATIC);
  motor4_PID.SetMode(AUTOMATIC);

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

  //Input=encoder1.get_rpm();
  //Setpoint=speed.data;
  //motor1_PID.Compute();

  //analogWrite(MOTOR1A, Output);


  Input1=encoder1.get_rpm();
  motor1_PID.Compute();
  motor1.setSpeed(Output1);
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
  Setpoint1=speed.data;
}

void motor2_callback(const std_msgs::Int16 &speed)
{
  Setpoint=speed.data;
  Input=encoder2.get_rpm();
  motor2_PID.Compute();
  motor2.setSpeed(Output);
}

void motor3_callback(const std_msgs::Int16 &speed)
{
  Setpoint=speed.data;
  Input=encoder3.get_rpm();
  motor3_PID.Compute();
  motor3.setSpeed(Output);
  
}

void motor4_callback(const std_msgs::Int16 &speed)
{

  Setpoint=speed.data;
  Input=encoder4.get_rpm();
  motor4_PID.Compute();
  motor4.setSpeed(Output);
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

