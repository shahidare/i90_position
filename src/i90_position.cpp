///////////////////////////////////////////////////////////////////////////////
//Source for i90_position node to estimate i90 position based on encoders		 //
//v1.0 																																			 //
//First creation 																														 //
//Huseyin Emre Erdem 																												 //
//08.08.2014 																																 //
///////////////////////////////////////////////////////////////////////////////

/*This node reads the encoder values from /drrobot_motor topic and estimates the
new position based on encoder values. The new position is published under the 
i90_pos topic. The type of the messages published is std_msgs::Float32MultiArray.*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "Float32.h"
#include "Float32MultiArray.h"

/*Prototypes*/
void encoderCallback(const drrobot_I90_player::MotorInfoArray motorInfoArray);

/*Main function*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "i90_position");//Create node called "i90_movement"
	ros::NodeHandle n;//Create nodehandler to modify features of the node
	ros::Publisher posPub = n.advertise<std_msgs::Float32MultiArray>("i90_pos", 1);
	ros::Subscriber encoderSub = n.subscribe("drrobot_motor", 1, encoderCallback);

	ros::Duration d = ros::Duration(2,0);
	ros::Rate loop_rate(2);
	geometry_msgs::Twist cmdvel_;//commands to be sent to i90

	while (ros::ok())
	{
		cmdvel_.linear.x = 0.1;
		ROS_INFO("Speed: %f", cmdvel_.linear.x);
		pub.publish(cmdvel_);
		// d.sleep();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void encoderCallback(const drrobot_I90_player::MotorInfoArray motorInfoArray){
	ROS_INFO("I heard: [%u]/t[%u]", motorInfoArray.motorInfos[0].encoder_pos, motorInfoArray.motorInfos[1].encoder_pos);
}

