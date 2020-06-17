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


volatile float pos_left = 0;	//Left motor encoder position
volatile float pos_right = 0; //Right motor encoder position





geometry_msgs::Pose2D pwm_output_msg;
geometry_msgs::Pose2D encoder_msg;
geometry_msgs::Vector3Stamped speed_msg;
geometry_msgs::Vector3 moto1_pid; 
geometry_msgs::Vector3 moto2_pid; 


ros::Publisher speed_pub("speed", &speed_msg);					//create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Publisher pwm_output_pub("pwm_output", &pwm_output_msg);
ros::Publisher encoder_pub("encoder", &encoder_msg);

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


	nh.spinOnce();
	//nh.loginfo("Publishing odometry");
}



void setup()
{
	nh.initNode();				    //init ROS node
	nh.getHardware()->setBaud(57600); //set baud for ROS serial communication
	//nh.getHardware()->setConnection(57600); //set baud for ROS serial communication
	nh.advertise(speed_pub);		    //prepare to publish speed in ROS topic



	while (!nh.connected())
	{
		nh.spinOnce();
	}
	
	nh.loginfo("ROBOT CONNECTED");
	delay(1);
}
int i = 0 ;
void loop()
{
	i= i +1 ;
	publishSpeed(i,10);
	nh.spinOnce();
	delay(100);

}
