#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/KeyboardInput.h"
#include "std_srvs/Empty.h"
#include <signal.h>

/*###PUBLISHER###*/
ros::Publisher pub;

/*###GLOBAL VARIABLES###*/
float front_th = 1.5;		// minimum front distance at which the robot stay from the walls
float k_angular = 10.0;		// constant angular velocity while the robot is turning
float k_linear = 0.4;		// constant linear velocity while the robot is turning
float default_vel = 2.0;	// default linear velocity while the robot is not facing any wall
float min_left;				// this variable will contain the minimum distance from the wall computed on the left of the robot
float min_right;			// this variable will contain the minimum distance from the wall computed on the right of the robot
float min_front;			// this variable will contain the minimum distance from the wall computed on in front of the robot
float min_front_l;			// this variable will contain the minimum distance from the wall computed on in front-left direction
float min_front_r;			// this variable will contain the minimum distance from the wall computed on in front-right direction
float laser[721];			// this float array will contain all the distances in every direction: from 0 (right of the robot) to 721 (left of the robot).

/*###MESSAGES###*/
geometry_msgs::Twist commanded_vel; // this is the declaration of a geomety_msgs::Twist type message

/*###CUSTOM SERVICE MESSAGE###*/
/*This is the declaration of a second_assignment::KeyboardInput type service message.
I created this custom service mesage for interpreting the keyboard input from the UI	*/
second_assignment::KeyboardInput user_input;

/*###CLIENTS###*/
ros::ServiceClient client;		   // Client that manages the keyboard inputs from UI.
ros::ServiceClient client_restart; // Client that manages the reset request from UI.

/*###FUNCTIONS###*/
float min_dist(int imin, int imax, float ranges[], float dist)
{

	/*This function computes the minimum value among all values in the 'ranges[]' array
	Arguments:
		1.	i_min (int): smaller index from which the computation should start
		2. 	i_max (int): grater  index in which the computation should end
		3. 	ranges[] (float): this is the array in which the computation must be done
	Return value:
		1.	min (float): minimum value among all values in the 'ranges[]' array. */

	// float min = 10;
	float min = dist;
	for (int i = imin; i < imax; i++)
	{
		if (ranges[i] < min)
		{
			min = ranges[i];
		}
	}
	return min;
}

bool ui_input(second_assignment::KeyboardInput::Request &req, second_assignment::KeyboardInput::Response &res)
{

	/*This is the function that uses the Service 'service'. It reads the requests from client and
	change the global variable 'user_input.response.multiplier' that multiplies the velocities so that
	it makes the robot increase/decrease the velocity.
	1.	The keyborad key 'a' is used for increasing the multiplier and therefore also the robot velocity.
	2.	The keyborad key 's' is used for decreasing the multiplier and therefore also the robot velocity.
	3.	The keyborad key 'q' is used for exiting the current node and the user interface node.
	4.	The keyborad key 'r' is used for restarting the robot from its initial position but also for resetting his initial velocity.

	This is a bool function, it returns true every time that one of the inputs above is received. */

	if (req.input == 'a')
	{
		user_input.response.multiplier += 0.5;
	}
	else if (req.input == 's')
	{
		user_input.response.multiplier -= 0.5;
	}
	else if (req.input == 'q')
	{
		kill(getpid(), SIGINT); // Kill the current process. (I included the <signnal.h> library!)
	}
	else if (req.input == 'r')
	{
		user_input.response.multiplier = 1.0;
	}
	return true;
}

void Drive(){
	
	min_left=min_dist(620, 720, laser, 10.0);
	min_right=min_dist(0, 100, laser, 10.0);
	min_front=min_dist(300, 420 , laser, 50.0);
	min_front_l=min_dist(450, 510, laser, 20.0);
	min_front_r=min_dist(170, 230, laser, 20.0);

	if(min_front<front_th){		
		if(min_left<min_right){		
			if(min_front_r<min_front_l){
				std::cout<< "Turning a little bit right...\n";
				commanded_vel.angular.z = -k_angular;
				commanded_vel.linear.x = k_linear;
			}
			else{
				std::cout<< "Turning right...\n";
				commanded_vel.angular.z = -k_angular;
				commanded_vel.linear.x = k_linear*user_input.response.multiplier;
			}
		}
		else if(min_right<min_left){	
			if(min_front_r>min_front_l){
				std::cout<< "Turning a little bit left...\n";
				commanded_vel.angular.z = k_angular;
				commanded_vel.linear.x = k_linear;
			}
			else{
				std::cout<< "Turning left...\n";
				commanded_vel.angular.z = k_angular;
				commanded_vel.linear.x = k_linear*user_input.response.multiplier;
			}
		}
		else {
			std::cout<< "Not Implemented...\n";
		}
	}
	else{		
		std::cout<< "Going straight...\n";
		commanded_vel.linear.x=(default_vel)*user_input.response.multiplier;
		commanded_vel.angular.z = 0.0;
	}
	pub.publish(commanded_vel);	
}

void ControllerCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

	/*This is the Callback function. Here the 'laser' array is filled with the values in the
	'ranges[]' field of the constant pointer to 'msg' that points to a 'sensor_msgs' object. 'ranges[]' contains
	information about the distances of the obstacles. Then the Drive(float, float, float, float[]) funciton is called.*/

	for (int i = 0; i < 721; i++)
	{
		laser[i] = msg->ranges[i];
	}

	Drive();
}

int main(int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "control");
	ros::NodeHandle node;

	// Initialize the clients
	client = node.serviceClient<second_assignment::KeyboardInput>("/keyboard_input"); // this is for keyboard input from UI node.
	client = node.serviceClient<std_srvs::Empty>("/reset_positions");					// this is for restarting the robot position.

	// Initialize the server
	ros::ServiceServer service = node.advertiseService("/keyboard_input", ui_input);

	// Define the publisher
	pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); // I published the velocity on the 'cmd_vel' topic
	user_input.response.multiplier = 1.0;							// At first the velocity multiplier must be 1.0

	// Initialize and define the subscriber
	ros::Subscriber sub = node.subscribe("/base_scan", 1000, ControllerCallback);

	// Loop
	ros::spin();

	return 0;
}
