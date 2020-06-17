#include <Arduino.h>
#include <Wire.h>
#include <PID.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <ros/time.h>
#include <config.h>
#include <kinematics.h>
#include <motor.h>

ros::NodeHandle nh;
unsigned long g_prev_command_time = 0;



PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

controller motor1_controller(controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
controller motor2_controller(controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); 

kinematics kinemati(kinematics::BASE, MAX_RPM, WHEEL_DIAMETER, 0, WHEEL_BASE);

float g_req_linear_vel_x = 0;
float g_req_angular_vel_z = 0;

volatile float pos_left = 0;	//Left motor encoder position
volatile float pos_right = 0; //Right motor encoder position


//function that will be called when receiving command from host
void CMDCallBack(const geometry_msgs::Twist &cmd_vel)
{
	g_req_linear_vel_x = cmd_vel.linear.x; //Extract the commanded linear speed from the message
	g_req_angular_vel_z = cmd_vel.angular.z; //Extract the commanded angular speed from the message
	g_prev_command_time = millis();
}

void PIDCallback(const geometry_msgs::Vector3 &pid)
{
    //callback function every time PID constants are received from lino_pid for tuning
    //this callback receives pid object where P,I, and D constants are stored
    motor1_pid.updateConstants(pid.x, pid.y, pid.z);
    motor2_pid.updateConstants(pid.x, pid.y, pid.z);
}



geometry_msgs::Pose2D pwm_output_msg;
geometry_msgs::Pose2D encoder_msg;
geometry_msgs::Vector3Stamped speed_msg;
geometry_msgs::Vector3 moto1_pid; 
geometry_msgs::Vector3 moto2_pid; 

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", CMDCallBack); //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Subscriber<geometry_msgs::Vector3> pid_constant("pid_constant", PIDCallback); //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)

ros::Publisher speed_pub("speed", &speed_msg);					//create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Publisher pwm_output_pub("pwm_output", &pwm_output_msg);
ros::Publisher encoder_pub("encoder", &encoder_msg);

template <typename T>
int sgn(T val)
{
	return (T(0) < val) - (val < T(0));
}

//Left motor encoder counter
void encoderLeftMotor()
{
	if (digitalRead(MOTOR1_ENCODER_A) == digitalRead(MOTOR1_ENCODER_B))
		pos_left++;
	else
		pos_left--;
}

//Right motor encoder counter
void encoderRightMotor()
{
	if (digitalRead(MOTOR2_ENCODER_A) == digitalRead(MOTOR2_ENCODER_B))
		pos_right--;
	else
		pos_right++;
}
//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(float current_rpm1,float current_rpm2)
{
	speed_msg.header.stamp = nh.now();	   //timestamp for odometry data
	speed_msg.vector.x = current_rpm1;  //left wheel speed (in m/s)
	speed_msg.vector.y = current_rpm2; //right wheel speed (in m/s)
	speed_msg.vector.z = LOOPTIME / 1000; //looptime, should be the same as specified in LOOPTIME (in s)
	speed_pub.publish(&speed_msg);

	encoder_msg.x = pos_left;
	encoder_msg.y = pos_right;
	encoder_pub.publish(&encoder_msg);


	//nh.spinOnce();
	//nh.loginfo("Publishing odometry");
}

void moveBase()
{
	kinematics::rpm req_rpm = kinemati.getRPM(g_req_linear_vel_x, 0, g_req_angular_vel_z);
	int current_rpm1 = pos_left/ENCODER_PULSE;
	int current_rpm2 = pos_right/ENCODER_PULSE;

	motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    	motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
	    

	kinematics::velocities current_vel;

     current_vel = kinemati.getVelocities(current_rpm1, current_rpm2, 0,0);
	
	publishSpeed(current_rpm1,current_rpm2);

}
void stopBase()
{
	g_req_linear_vel_x  = 0;
	g_req_angular_vel_z = 0;
}

void printDebug()
{
	char buffer[50];
	double current_encoder_left  = pos_left/ENCODER_PULSE;
	double current_encoder_right = pos_right/ENCODER_PULSE;
    	sprintf (buffer, "Encoder FrontLeft  : %lf",current_encoder_left );
    	nh.loginfo(buffer);
    	sprintf (buffer, "Encoder FrontRight : %lf", current_encoder_right );
    	nh.loginfo(buffer);
}

void setup()
{
	nh.initNode();				    //init ROS node
	nh.getHardware()->setBaud(57600); //set baud for ROS serial communication
	//nh.getHardware()->setConnection(57600); //set baud for ROS serial communication
	nh.subscribe(cmd_vel);		    //suscribe to ROS topic for velocity commands
	nh.subscribe(pid_constant);		    //suscribe to ROS topic for velocity commands
	nh.advertise(speed_pub);		    //prepare to publish speed in ROS topic
	nh.advertise(encoder_pub);
	nh.advertise(pwm_output_pub);



	// Define the rotary encoder for left motor
	pinMode(MOTOR1_ENCODER_A, INPUT);
	pinMode(MOTOR1_ENCODER_B, INPUT);
	digitalWrite(MOTOR1_ENCODER_A, HIGH); // turn on pullup resistor
	digitalWrite(MOTOR1_ENCODER_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_A), encoderLeftMotor, RISING);

	// Define the rotary encoder for right motor
	pinMode(MOTOR2_ENCODER_A, INPUT);
	pinMode(MOTOR2_ENCODER_B, INPUT);
	digitalWrite(MOTOR2_ENCODER_A, HIGH); // turn on pullup resistor
	digitalWrite(MOTOR2_ENCODER_B, HIGH);
	attachInterrupt(digitalPinToInterrupt(MOTOR2_ENCODER_A), encoderRightMotor, RISING);

	while (!nh.connected())
	{
		nh.spinOnce();
	}
	
	nh.loginfo("ROBOT CONNECTED");
	delay(1);
}

void loop()
{
	static unsigned long prev_control_time = 0;
	static unsigned long prev_debug_time = 0;

	//this block drives the robot based on defined rate
	if ((millis() - prev_control_time) >= LOOPTIME)
	{
		moveBase();
		prev_control_time = millis();
	}

	//this block stops the motor when no command is received
	if ((millis() - g_prev_command_time) >= 400)
	{
		stopBase();
	}

	//this block displays the encoder readings. change DEBUG to 0 if you don't want to display
	if (DEBUG)
	{
		if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
		{
			printDebug();
			prev_debug_time = millis();
		}
	}
	nh.spinOnce();

}
