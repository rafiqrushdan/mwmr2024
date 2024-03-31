#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include "motor.h"
#include "encoder.h"

#include "config.h"

ros::NodeHandle nh;
ros::init(argc, argv, "controller_node");


/////////////PID///////////////
  #include <PID_v1.h>
  //Define Variables we'll be connecting to

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;

//Specify the links and initial tuning parameters
double Kp1=2, Ki1=0, Kd1=0;
double Kp2=2, Ki2=5, Kd2=1;
double Kp3=32, Ki3=5, Kd3=1;
double Kp4=2, Ki4=5, Kd4=1;

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

//declare the function protoype
void encoder1Update();
void encoder2Update();
void encoder3Update();
void encoder4Update();

void PIDcallback();
void computePID();


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

  PIDcallback();


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

  computePID();


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
  Setpoint2=speed.data;
 
}

void motor3_callback(const std_msgs::Int16 &speed)
{
  Setpoint3=speed.data;

}

void motor4_callback(const std_msgs::Int16 &speed)
{

  Setpoint4=speed.data;

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

  void PIDcallback() //this function will handle the callback of PID params
  {
    nh.param("controller/Kp1", Kp1, 2.1);
    nh.param("controller/Ki1", Ki1, 0.8);
    nh.param("controller/Kd1", Kd1, 0.5);

    nh.param("controller/Kp2", Kp2, 2.0);
    nh.param("controller/Ki2", Ki2, 5.0);
    nh.param("controller/Kd2", Kd2, 1.0);

    nh.param("controller/Kp3", Kp3, 32.0);
    nh.param("controller/Ki3", Ki3, 5.0);
    nh.param("controller/Kd3", Kd3, 1.0);

    nh.param("controller/Kp4", Kp4, 2.0);
    nh.param("controller/Ki4", Ki4, 5.0);
    nh.param("controller/Kd4", Kd4, 1.0);
  }

  void computePID() //this function will compute all the PID
  {
  Input1=encoder1.get_rpm();
  motor1_PID.Compute();
  motor1.setSpeed(Output1);  

  Input2=encoder2.get_rpm();
  motor2_PID.Compute();
  motor2.setSpeed(Output2);

  Input3=encoder3.get_rpm();
  motor3_PID.Compute();
  motor3.setSpeed(Output3);

  Input4=encoder4.get_rpm();
  motor4_PID.Compute();
  motor4.setSpeed(Output4);
  }