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


//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter


const int PIN_L_IN1 = 24;
const int PIN_L_IN2 = 25;
const int PIN_L_PWM = 6;
const int PIN_L_ENCOD_A_MOTOR = 2;               //A channel for encoder of left motor
const int PIN_L_ENCOD_B_MOTOR = 4;               //B channel for encoder of left motor

const int PIN_R_IN1 = 30;
const int PIN_R_IN2 = 31;
const int PIN_R_PWM = 7;
const int PIN_R_ENCOD_A_MOTOR = 3;              //A channel for encoder of right motor
const int PIN_R_ENCOD_B_MOTOR = 18;              //B channel for encoder of right motor

unsigned long lastMilli = 0;
const double radius = 0.07;                   //Wheel radius, in m
const double wheelbase = 0.325;               //Wheelbase, in m

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s

const double max_speed = 0.4;                 //Max speed in m/s

int PWM_leftMotor = 0;                     //PWM command for left motor
int PWM_rightMotor = 0;                    //PWM command for right motor

unsigned int encoder_left = 0;
unsigned int encoder_right = 0;


// PID Parameters
const double PID_left_param[] = { 0.29, 0, 0.0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0.262, 0, 0.0 }; //Respectively Kp, Ki and Kd for right motor PID

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position


PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left  = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}
 
geometry_msgs::Pose2D pwm_output_msg; 
geometry_msgs::Pose2D encoder_msg; 
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)

ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Publisher pwm_output_pub("pwm_output", &pwm_output_msg);
ros::Publisher encoder_pub("encoder", &encoder_msg);


void setup() {
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  nh.advertise(encoder_pub);
  nh.advertise(pwm_output_pub);


  //setting motor speeds to zero
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_L_PWM, OUTPUT);

  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);



  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);

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
}

void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  {  
    lastMilli = millis();

    if (abs(pos_left) < 5) {                                                  //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left = ((pos_left / 480) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of left wheel             !!!! 990 encoder total pulse coun
    }

    if (abs(pos_right) < 5) {                                                 //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
      speed_act_right = ((pos_right / 480) * 2 * PI) * (1000 / LOOPTIME) * radius; // calculate speed of right wheel          !!!! 990 encoder total pulse coun
    }

    encoder_left = encoder_left + pos_left;
    encoder_right = encoder_right + pos_right;

    if(encoder_left>32000) encoder_left = 0; 
    if(encoder_right>32000) encoder_right = 0; 
    

    pos_left = 0;
    pos_right = 0;

    speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    PID_leftMotor.Compute();                                                 // compute PWM value for left motor
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*0.0882)/0.00235) + (speed_cmd_left/0.00235), -255, 255);

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      analogWrite(PIN_L_PWM, 0);
    }
    else if (speed_req_left == 0) {                       //Stopping
      analogWrite(PIN_L_PWM, 0);
    }
    else if (PWM_leftMotor > 0) {                         //Going forward
      analogWrite(PIN_L_PWM, PWM_leftMotor);
      digitalWrite(PIN_L_IN1, HIGH);
      digitalWrite(PIN_L_IN2, LOW);
    }
    else {                                               //Going backward
      analogWrite(PIN_L_PWM, abs(PWM_leftMotor));
      digitalWrite(PIN_L_IN1, LOW);
      digitalWrite(PIN_L_IN2, HIGH);
    }

    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    PID_rightMotor.Compute();                                                 // compute PWM value for right motor
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*0.0882)/0.00235) + (speed_cmd_right/0.00235), -255, 255); 

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      analogWrite(PIN_R_PWM, 0);
    }
    else if (speed_req_right == 0) {                      //Stopping
      analogWrite(PIN_R_PWM, 0);
    }
    else if (PWM_rightMotor > 0) {                        //Going forward
      analogWrite(PIN_R_PWM, PWM_rightMotor);
      digitalWrite(PIN_R_IN1, HIGH);
      digitalWrite(PIN_R_IN2, LOW);
    }
    else {                                                //Going backward
      analogWrite(PIN_R_PWM, abs(PWM_rightMotor));
      digitalWrite(PIN_R_IN1, LOW);
      digitalWrite(PIN_R_IN2, HIGH);
    }
 
    noCommLoops++;
    if (noCommLoops == 65535) {
      noCommLoops = noCommLoopMax;
    }

    publishSpeed();   //Publish odometry on ROS topic
  }
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed() {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = LOOPTIME / 1000;   //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);


  encoder_msg.x= encoder_left;
  encoder_msg.y= encoder_right;
  encoder_pub.publish( &encoder_msg);

  pwm_output_msg.x = PWM_leftMotor;
  pwm_output_msg.y = PWM_rightMotor;
  pwm_output_pub.publish(&pwm_output_msg);
 
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
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
