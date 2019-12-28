#include <TimerOne.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

//PINS NAMES SHOULD BE CAPITAL LETTER 
const int IN1=24;
const int IN2=25;
const int L_PWM=6;
const int L_ENCODER=2;
const int L_ENCODER_1=18;

const int IN3=30;
const int IN4=31;
const int R_PWM=7;
const int R_ENCODER=3;
const int R_ENCODER_1=19;

//VARIABLES IN HERE 
double kp = .07;
double ki = .00;
double kd = .03;
 
double l_error;
double l_lastError;
double l_input;
double l_out;

double l_cumError, l_rateError;

double r_error;
double r_lastError;
double r_input;
double r_out;

double r_cumError, r_rateError;

volatile double r_encCount = 0;
volatile double l_encCount = 0;
double l_rpm=0, r_rpm=0;
double r_w=0, l_w=0;


double pub_l_out=0;
double pub_r_out=0;

double wheel_rad = 0.0325, wheel_sep = 0.295;
double speed_ang=0, speed_lin=0;

void subsCMD( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  r_w = 100*((speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad)));
  l_w = 100*((speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad)));
  //Serial.println(r_w); 
}
ros::NodeHandle nh;
 

geometry_msgs::Twist vel_msg;
std_msgs::String str_msg;
std_msgs::String publish_pwm_msg;
ros::Publisher chatter("pwm_publiser", &str_msg);
ros::Publisher pub_velocity("pwm_publiser_1", &vel_msg);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &subsCMD );


void setup(){ 

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(L_ENCODER, INPUT);
  
  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(R_ENCODER, INPUT);
  
      
  attachInterrupt(digitalPinToInterrupt(L_ENCODER), leftEncoderEvent , RISING);      //pin 2 encoder
  attachInterrupt(digitalPinToInterrupt(R_ENCODER), rightEncoderEvent, RISING);     //pin 3 encoder 


  //attachInterrupt(digitalPinToInterrupt(L_ENCODER_1), leftEncoderEvent_1  , RISING);      //pin 2 encoder
  //attachInterrupt(digitalPinToInterrupt(R_ENCODER_1), rightEncoderEvent_1 , RISING);     //pin 3 encoder 
  
  Timer1.initialize(100000); 
  Timer1.attachInterrupt(timerPID); 

  //LEFT MOTOR 
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);

  //RIGHT MOTOR
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  
  nh.initNode();
  nh.subscribe(sub);      //subscribe cmd_vel topic 
  nh.advertise(chatter);  //publish pwm with pwm_output topic 
  
  //Serial.begin(57600);
}    

void loop()
{
   
  char chr_l_out[12];
  itoa(pub_l_out, chr_l_out,10);
  char chr_r_out[12];
  itoa(pub_r_out, chr_r_out,10);
  str_msg.data = chr_l_out;
  chatter.publish( &str_msg);
  

  /*
  vel_msg.linear.x = pub_l_out;
  vel_msg.linear.y = pub_r_out;
  pub_velocity.publish(&vel_msg);
  */
  nh.spinOnce();
  delay(1);
}


void timerPID()        // interrupt service routine - tick every 0.1sec
{   
  l_rpm = 600*(l_encCount/16);  //calculate motor speed, unit is rpm
  l_encCount=0;
  l_error = l_w - l_rpm; 
  l_cumError += l_error; //* elapsedTime;                
  l_rateError = (l_error - l_lastError);///elapsedTime;
  double l_out = kp * l_error + ki * l_cumError + kd * l_rateError; 
  analogWrite(L_PWM , abs(2*l_out));
  l_lastError = l_error; 
  pub_l_out = 2*l_out;     
  
  if(l_out>0)
  {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  }
  else if(l_out<0)  
  {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);      
  }

 
  r_rpm = 600*(r_encCount/16);  //calculate motor speed, unit is rpm
  r_encCount = 0;
  r_error = r_w - r_rpm; 
  r_cumError += r_error; //* elapsedTime;                
  r_rateError = (r_error - r_lastError);///elapsedTime;                                 
  double r_out = kp * r_error + ki * r_cumError + kd * r_rateError; 
  analogWrite(R_PWM , abs(2*r_out));   
  r_lastError = r_error;      
  pub_r_out = 2*r_out;     
  
  if(r_out>0)
  {
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  }
  else if(r_out<0)   
  {
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);      
  }
}

void leftEncoderEvent() {
  if(l_out>0)       l_encCount ++;
  else if(l_out<0)   l_encCount --;
}

void rightEncoderEvent() {
  if(r_out>0) r_encCount ++;
  else if(r_out<0) r_encCount --;
}

void leftEncoderEvent_1() {
  if(l_out>0)       l_encCount ++;
  else if(l_out<0)   l_encCount --;
}

void rightEncoderEvent_1() {
  if(r_out>0) r_encCount ++;
  else if(r_out>0) r_encCount --;
}                                                                            
