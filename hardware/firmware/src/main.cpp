#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int16.h>

#include "motor.h"
#include "encoder.h"

#include "config.h"

ros::NodeHandle nh;

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
Encoder encoder1(ENC1A, ENC1B, CPR, true);
Encoder encoder2(ENC2A, ENC2B, CPR, false);
Encoder encoder3(ENC3A, ENC3B, CPR, false);
Encoder encoder4(ENC4A, ENC4B, CPR, false);

std_msgs::Int16 enc1;
std_msgs::Int16 enc2;
std_msgs::Int16 enc3;
std_msgs::Int16 enc4;

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
  attachInterrupt(digitalPinToInterrupt(ENC2A), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2B), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3A), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3B), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4A), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4B), encoder1Update, CHANGE);

  prev_time = millis();
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
}

void send_encoder_data()
{
  enc1.data = encoder1.get_count();
  enc2.data = encoder2.get_count();
  enc3.data = encoder3.get_count();
  enc4.data = encoder4.get_count();
  pub1.publish(&enc1);
  pub2.publish(&enc2);
  pub3.publish(&enc3);
  pub4.publish(&enc4);
}

void motor1_callback(const std_msgs::Int16 &speed)
{
  motor1.setSpeed(speed.data);
}

void motor2_callback(const std_msgs::Int16 &speed)
{
  motor2.setSpeed(speed.data);
}

void motor3_callback(const std_msgs::Int16 &speed)
{
  motor3.setSpeed(speed.data);
}

void motor4_callback(const std_msgs::Int16 &speed)
{
  motor4.setSpeed(speed.data);
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