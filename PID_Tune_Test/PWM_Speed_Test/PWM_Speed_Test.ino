#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/time.h>
#include <ArduinoHardware.h>


#define LOOPTIME                      100  

const int PIN_L_IN1 = 24;
const int PIN_L_IN2 = 25;
const int PIN_L_PWM = 6;
const int PIN_L_ENCOD_A_MOTOR = 2;               //A channel for encoder of left motor
const int PIN_L_ENCOD_B_MOTOR = 4;               //B channel for encoder of left motor

const int PIN_R_IN1 = 30;
const int PIN_R_IN2 = 31;
const int PIN_R_PWM = 7;
const int PIN_R_ENCOD_A_MOTOR = 3;              //A channel for encoder of right motor
const int PIN_R_ENCOD_B_MOTOR = 18;     


volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position
unsigned long lastMilli = 0;
const double radius = 0.07; 

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;    
int PWM_value = 0;                         

ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  PWM_value = cmd_vel.linear.x*100;                                     //Extract the commanded linear speed from the message
  analogWrite(PIN_L_PWM, PWM_value);
  analogWrite(PIN_R_PWM, PWM_value);

}


geometry_msgs::Vector3Stamped speed_msg;
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Publisher speed_pub("speed", &speed_msg);             

void setup() {
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic


  //setting motor speeds to zero
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);

  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);

    // Define the rotary encoder for left motor
  pinMode(PIN_L_ENCOD_A_MOTOR, INPUT);
  pinMode(PIN_L_ENCOD_B_MOTOR, INPUT);
  digitalWrite(PIN_L_ENCOD_A_MOTOR, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_L_ENCOD_B_MOTOR, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_R_ENCOD_A_MOTOR, INPUT);
  pinMode(PIN_R_ENCOD_B_MOTOR, INPUT);
  digitalWrite(PIN_R_ENCOD_A_MOTOR, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_R_ENCOD_B_MOTOR, HIGH);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENCOD_A_MOTOR), encoderRightMotor, RISING);

  digitalWrite(PIN_L_IN1, HIGH);
  digitalWrite(PIN_L_IN2, LOW);
  digitalWrite(PIN_R_IN1, HIGH);
  digitalWrite(PIN_R_IN2, LOW);

}

void loop() {
nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  {
    lastMilli = millis();

    speed_act_left =  ((pos_left  / 480) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel             !!!! 990 encoder total pulse coun
    speed_act_right = ((pos_right / 480) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel          !!!! 990 encoder total pulse coun
    pos_left = 0 ;
    pos_right = 0 ; 
    
    publishSpeed();  
  }  
}


//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed() {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = PWM_value;   //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);

 
  nh.spinOnce();
  nh.loginfo("Publishing SPEED for PWM");
}

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_L_ENCOD_A_MOTOR) == digitalRead(PIN_L_ENCOD_B_MOTOR)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_R_ENCOD_A_MOTOR) == digitalRead(PIN_R_ENCOD_B_MOTOR)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
