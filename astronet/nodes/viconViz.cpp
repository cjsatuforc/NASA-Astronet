/*
Node to visualize Vicon Messages by moving the 
human model with respect to the motion capture 
messages capturing wrist and head motions

Coordinate Conventions:
when you move towards +x in the workspace, the 
gazebo model towards +x. When the model moves 
towards +y, the gazebo model moves towards +y.
*/

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TransformStamped.h>
#define headHeight 1.85
#define shoulderHeight 1.75
#define scaleX 0.7
#define scaleY 1
#define scaleZ 0.7

using namespace ros;

// subscriber required for every model
Subscriber sub_head;
Subscriber sub_left;
Subscriber sub_right;

// publisher required for every model
Publisher pub_head;
Publisher pub_torso;
Publisher pub_left;
Publisher pub_right;

// Single client required for storing all values
ServiceClient client;

// declare initial values for the states of each model
float head_x = -3.75, head_y = 0, head_z = 5.25;
float torso_x = -3.75, torso_y = 0, torso_z = 4.75;
float left_x = -3.25, left_y = 0.25, left_z = 5;
float right_x = -3.25, right_y = -0.25, right_z = 5;

// Client to get initial states of all models
void get_states() {
	gazebo_msgs::GetModelState modelstate;

	// store head position
	modelstate.request.model_name = "head";
	client.call(modelstate);
	head_x = modelstate.response.pose.position.x;
	head_y = modelstate.response.pose.position.y;
	head_z = modelstate.response.pose.position.z;

	// store torso position
	modelstate.request.model_name = "torso";
	client.call(modelstate);
	torso_x = modelstate.response.pose.position.x;
	torso_y = modelstate.response.pose.position.y;
	torso_z = modelstate.response.pose.position.z;

	// store left arm position
	modelstate.request.model_name = "left";
	client.call(modelstate);
	left_x = modelstate.response.pose.position.x;
	left_y = modelstate.response.pose.position.y;
	left_z = modelstate.response.pose.position.z;

	// store right arm position
	modelstate.request.model_name = "right";
	client.call(modelstate);
	right_x = modelstate.response.pose.position.x;
	right_y = modelstate.response.pose.position.y;
	right_z = modelstate.response.pose.position.z;
}




// Model Publishers
void publish_torso(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = torso_x + scaleX*x;
	msg.pose.position.y = torso_y + scaleY*y;
	msg.pose.position.z = torso_z; // setting average value of head_z as origin
	pub_torso.publish(msg);
}


void publish_head(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = head_x + scaleX*x;
	msg.pose.position.y = head_y + scaleY*y;
	msg.pose.position.z = head_z + scaleZ*(z - headHeight); // setting average value of head_z as origin
	pub_head.publish(msg);

	gazebo_msgs::ModelState torso_msg;
	torso_msg.model_name = "torso";
	publish_torso(torso_msg, x, y, z);
}


void publish_left(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = left_x + scaleX*x;
	msg.pose.position.y = left_y + scaleY*y;
	msg.pose.position.z = left_z + scaleZ*(z - shoulderHeight);
	pub_left.publish(msg);
}


void publish_right(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = right_x + scaleX*x;
	msg.pose.position.y = right_y + scaleY*y;
	msg.pose.position.z = right_z + scaleZ*(z - shoulderHeight);
	pub_right.publish(msg);
}



// Vicon Data Listeners
void listener_head(const geometry_msgs::TransformStamped &msg) {
	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;
	float z = msg.transform.translation.z;

	gazebo_msgs::ModelState head_msg;
	head_msg.model_name = "head";

	publish_head(head_msg, x, y, z);
}

void listener_left(const geometry_msgs::TransformStamped &msg) {
	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;
	float z = msg.transform.translation.z;

	gazebo_msgs::ModelState left_msg;
	left_msg.model_name = "left";
	publish_left(left_msg, x, y, z);
}

void listener_right(const geometry_msgs::TransformStamped &msg) {
	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;
	float z = msg.transform.translation.z;

	gazebo_msgs::ModelState right_msg;
	right_msg.model_name = "right";
	publish_right(right_msg, x, y, z);
}



// Main loop to orchestrate publishers and subscribers
int main(int argc, char **argv) {
	init(argc, argv, "viconViz");
	NodeHandle nh;

	sub_head = nh.subscribe("/vicon/William/William", 10000, &listener_head);
	sub_left = nh.subscribe("/vicon/Left_Arm/Left_Arm", 10000, &listener_left);
	sub_right = nh.subscribe("/vicon/Right_Arm/Right_Arm", 10000, &listener_right);

	pub_head = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_torso = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_left = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_right = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	/*
	capture and store initial positions of all models
	to be fixed for delayed launch. Node works fine 
	when launched standalone but produces readings of 
	(0,0,0) when launched from launch file.
	*/
	// get_states();

	spin();
}