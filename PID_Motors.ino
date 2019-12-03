#include <Timer One.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

const int IN1=24;
const int IN2=25;
const int ENL=6;
const int IN3=28;
const int IN4=29;
const int ENR=7;

double kp = .07;
double ki = .01;
double kd = .03;
 
double l_error;
double l_lastError;
double l_input;
double l_out;
double l_Setpoint;
double l_cumError, l_rateError;

double r_error;
double r_lastError;
double r_input;
double r_out;
double r_Setpoint;
double r_cumError, r_rateError;

volatile double r_encCount = 0;
volatile double l_encCount = 0;
double l_rpm=0, r_rpm=0;
double r_w=0, l_w=0;

double wheel_rad = 0.0325, wheel_sep = 0.295;
double speed_ang=0, speed_lin=0;

void subsCMD( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  r_w = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  l_w = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
   
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &subsCMD );


void setup(){
  Setpoint = 5000;   
  MSetpoint = 5000;   
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENL, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(ENR, OUTPUT);
      
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(3), rightEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(2), leftEncoderEvent, CHANGE);
  Timer1.initialize(100000); 
  Timer1.attachInterrupt(timerPID); 
  digitalWrite(30,HIGH);
  digitalWrite(31,LOW);
  
  digitalWrite(24,HIGH);
  digitalWrite(25,LOW); 
  Serial.begin(9600);
}    

void timerPID()        // interrupt service routine - tick every 0.1sec
{   
  l_rpm = 60.0*(l_encCount/64)/0.1;  //calculate motor speed, unit is rpm
  l_encCount=0;
  l_error = l_w - l_rpm; 
  l_cumError += l_error; //* elapsedTime;                
  l_rateError = (l_error - l_lastError);///elapsedTime;
  double l_out = kp * l_error + ki * l_cumError + kd * l_rateError; 
  analogWrite(ENL , l_out);  
  l_lastError = l_error;                     

  Serial.println(l_out); 

  r_rpm = 60.0*(r_encCount/64)/0.1;  //calculate motor speed, unit is rpm
  r_encCount = 0;
  r_error = r_w - r_rpm; 
  r_cumError += r_error; //* elapsedTime;                
  r_rateError = (r_error - r_lastError);///elapsedTime;                                 
  double r_out = kp * r_error + ki * r_cumError + kd * r_rateError; 
  analogWrite(ENR , r_out);   
  r_lastError = r_error;          
}
void loop()
{
  Serial.print("  ");            
}
void leftEncoderEvent() {
  l_encCount ++ ;
}

void rightEncoderEvent() {
  r_encCount ++ ;
}