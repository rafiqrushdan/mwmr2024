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
Encoder encoder1(15, 2, 250, false);

std_msgs::Int16 enc1;

ros::Publisher pub1("enc1", &enc1);

void encoder1Update();

void send_encoder_data();

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

  delay(100);

  attachInterrupt(digitalPinToInterrupt(15), encoder1Update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), encoder1Update, CHANGE);
}

void loop()
{
  nh.spinOnce();
  send_encoder_data();
  delay(20);
}

void send_encoder_data()
{
  enc1.data = encoder1.get_count();
  pub1.publish(&enc1);
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