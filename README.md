## Research Track 1, second assignment

================================

### Introduction

--------------------------------

The second assignment of the research track 1 class is about a robot with laser sensor that has is simulated using the ROS framework and the C++ programing language. The robot is effectively represented by a blue cube, and our task is to design a controller to drive the robot through given a racetrack. Additionally, a User Interface is designed to continuously watch for user input. The user may ask to increase or decrease the robot's velocity or to return it to its starting point. Consequently, there were two distinct nodes required:

* The controlling node, that I called **control.cpp**
* The UI node, that I called **ui.cpp**

Here's some pictures that show the simulation enviroment provided us by profesor [Carmine Recchiuto](https://github.com/CarmineD8):
<p align="center">
 <img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos,%20gifs%20%20and%20images/Robot.JPG" height=320 width=380>
</p>
<p align="center">
 <img src="https://github.com/FraPagano/RT_Assignment_2/blob/main/Videos,%20gifs%20%20and%20images/Circuit.JPG" height=320 width=380>
</p>

The two nodes I created are quite simple: in the controller node the idea is to constantly check for data from laser sensors which the robot is equipped with and make turning decisions on the base of such data. Velocities were set by implementing a publish–subscribe messaging pattern. On the other hand, the UI node waits for inputs from the user and by using a custom Service it modifies the robot velocity depending on the pressed key. Some additional features were added on the UI node. The custom service message (`KeyboardInput.srv`) that I created is very simple: the `request` field is a char that will represent the user keyboard input. The `response` field is a float because it has to represent a number that will be the robot velocity multiplier.

### Code description
---------------------------

#### Controlling node

Some important global object were instantiated here such as:

 1. A `geometry_msgs::Twist commanded_vel` This object is a Twist type of the class `geometry_msgs` and in particualr is a message used for setting both the robot's linear and angular default velocities;
 2. `second_assignment::KeyboardInput user_input`. This is the custom service message on purpose designed for the user keyboard inputs;
 3. A `ros::ServiceClient` for the velocity changes, handled by the UI node; 
 4. Another `ros::ServiceClient` for the restart request, handled by the UI node too.

The controlling node is the one that set both the robot's linear and angular velocities and elaborates data from laser sensors. Here, a turn decison method was implemented but also a Server that collects the Client (in the UI node) requests.
The functions that I created are:

- float `min_dist(int imin, int imax, float ranges[])`: This function computes the minimum value among all values in the `ranges[]` array. The arguments are: 
		1.	`i_min` (int): smaller index from which the computation should start
		2. 	`i_max` (int): grater  index in which the computation should end    
		3. 	`ranges[]` (float): this is the array in which the computation must be done
	
It returns the minimum value among all values in the `ranges[]` array. The array's checked spans are: 

	 - Left side, corresponding to the 0-100 array span.
	 - Right side, corresponding to the 620-720 array span.
	 - Front direction, corresponding to the 300-420 array span.
	 - Front-left side, corresponding to the 450-510 array span.
	 - Front-right side, corresponding to the 170-230 array span

Thanks to the `min_dist(int imin, int imax, float ranges[], float dist)` function I could extrapolate the lowest distances among each span and therefore I could also let the robot make decisions based on the minimum distances. 
The velocity managing  was done through the `geometry_message::Twist` message. The fields *linear* and *angular* have been used to modify the linear and angular velocity of the robot, while the fields *x* and *z* define orientation axis on which the velocity is considered. Finally, the message is published.
	Arguments:
			1.	`min_left` (float): minimum distance from the wall on the left of the robot;
			2.	`min_right` (float): minimum distance from the wall on the right of the robot;
			3.	`min_front` (float): minimum distance from the wall in front of the robot.
			1. 	`ranges[]` (float): array in which the computation takes place.

 - `void Drive()`: This is the function that drives the robot into the circuit. This function will be called in the Callback function so that the instructions will be looped. The implementation logic is quite simple: the minimum distances around the robot are continuously updated. If the distace in front of the robot is less than 1.5 then a turning method is activated. If no walls closer than 1.5 are detected in the front direction the robot is simply going straight. In order to read all the data from the laser sensor which the robot is equipped with, I implemented a subscriber to the `"/base_scan"` topic. This topic uses a message of type [`sensor_msgs::LaserScan`](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) whose field called `ranges`  is an array of 721 elements. Such array provide us the distances from the walls in every direction around the robot. Element 0 give us the distance from the wall on the right side and the 721st element give us the distance on the left side. The logic is quite simple: if the distance on the right is less than the distance on the left, the robot is turning left. And the opposite is also true. Furthermore, the front left/right directions are checked in order to avoid the robot to bump into the wall when its velocity is high.
			
There are no return values for this funciton.

#### UI node

The User Interface node handles the user keyboard inputs. Here's a legend of the allowed commands:

 - *'a'* keyboard key is used for increasing the robot linear velocity;
 - *'s'* keyboard key is used for decreasing the robot linear velocity;
 - *'r'* keyboard key is used for resetting the robot position and linear velocity;
 - *'q'* keyboard key is used for quitting the application and terminates all the nodes.

This is a bool function, it returns `true` every time that one of the inputs above is received.

 - `bool ui_input(second_assignment::KeyboardInput::Request &req,
   second_assignment::KeyboardInput::Response &res)`:  This is the server function that reads the requests from clients and changes the global variable `user_input.response.multiplier` (that is the response field of the custom service message) that multiplies the default velocities so that it makes the robot velocity increase/decrease. In particular,
		1.	The keyborad key 'a' is used for increasing the multiplier and therefore also the robot velocity.
		2.	The keyborad key 's' is used for decreasing the multiplier and therefore also the robot velocity.
		3.	The keyborad key 'q' is used for terminating the processes.
		4.	The keyborad key 'r' is used for restarting the robot from its initial position but also for resetting his initial velocity.	 

##### Some important global object were instantiated such as:

 1. The custom service message `second_assignment::KeyboardInput user_input`: here the `request` field of this object is set.
 2. A `ros::ServiceClient` for handling the keyboard input; 
 3. Another `ros::ServiceClient` for handling the service that restart the robot position; 
 4. A `char` type that will contain the user inputs,
 5. Another `char` type that will contain inputs when the user presses 'q'; 

In the `main` function of the node a `while(ros::ok())` loop was created in order to constantly wait for user input. Whenever one of the above keyboard key is pressed, through `cin>>`, the user input is put in the `command` variable. If the command is a not allowed command, an error message is printed. Instead, if the user input is an allowed command, the `request` field of the custom service message `second_assignment::KeyboardInput user_input` is filled and the request is sent to the server. The cases in which the input are *'r'* and *'q'*  are a little bit different from other cases because in *'r'* case we call two services: the one for restarting the robot position and the other one for resetting the default velocity. For the *'q'* case, instead, some control for safety were implemented.

###  Installing and running 
----------------------

Here's some useful informations regarding running the simulator.
First of all, [xterm](https://it.wikipedia.org/wiki/Xterm), a standard terminal emulator, is needed. You can install xterm by entering the following commands in the terminal:
```
sudo apt update
sudo apt-get install xterm
```
I created a launch file in the _launch_ directory that executes three nodes at the same time:

 - The `stageros` node, that runs the simulation environment; 
 - The `control` node; 
 - The `ui` node.

You can run the program by entering the following command:

```
roslaunch second_assignment launch_nodes.launch 
```

The UI node will run on an xterm terminal. 

If any of the three node terminates the launch file will terminates all the nodes.

### Pseudocode
--------------------------------



### Results
--------------------------------

The final result is that the robot correctly runs around the circuit and, despite there are some things that could be improved in the future, I am satisfied with the work that I've done specially because that was my first approach with the ROS framework. 

In order to make you understand how my code works, I recorded this video:


https://user-images.githubusercontent.com/91267426/143897374-b2abea45-ce71-4e31-89b0-dc77842e4775.mp4






### Possible Improvements
--------------------------------
A possible improvement that can be implemented is certainly to avoid bumping the robot into the wall when we increase its speed a lot. Some controls were implemented such as the computation of the minimum distance in the  front left & right direction span. Nevertheless I measured that with a 3.5 times greater velocity than the default velocity the robot still bumps into the wall.







