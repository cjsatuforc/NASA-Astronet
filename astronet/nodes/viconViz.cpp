/*
Node to visualize Vicon Messages by moving the 
human model with respect to the motion capture 
messages capturing wrist and head motions
*/
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TransformStamped.h>

using namespace ros;

// subscriber required for every model
Subscriber sub_head;
Subscriber sub_left;
Subscriber sub_right;

// publisher required for every model
Publisher pub_head;
Publisher pub_left;
Publisher pub_right;

// Single client required for storing all values
ServiceClient client;

float head_x, head_y, head_z;
float left_x, left_y, left_z;
float right_x, right_y, right_z;

// Client to get initial states of all models
void get_states() {
	gazebo_msgs::GetModelState head_state;
	gazebo_msgs::GetModelState left_state;
	gazebo_msgs::GetModelState right_state;

	head_state.request.model_name = "head";
	left_state.request.model_name = "left";
	right_state.request.model_name = "right";

	// store head position
	client.call(head_state);
	head_x = head_state.response.pose.position.x;
	head_y = head_state.response.pose.position.y;
	head_z = head_state.response.pose.position.z;

	// store left arm position
	client.call(left_state);
	left_x = left_state.response.pose.position.x;
	left_y = left_state.response.pose.position.y;
	left_z = left_state.response.pose.position.z;

	// store right arm position
	client.call(right_state);
	right_x = right_state.response.pose.position.x;
	right_y = right_state.response.pose.position.y;
	right_z = right_state.response.pose.position.z;
}




// Model Publishers
void publish_head(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = head_x + x;
	msg.pose.position.y = head_y + y;
	msg.pose.position.z = head_z + z;
	pub_head.publish(msg);
}


void publish_left(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = left_x + x;
	msg.pose.position.y = left_y + y;
	msg.pose.position.z = left_z + z;
	pub_left.publish(msg);
}


void publish_right(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	msg.pose.position.x = right_x + x;
	msg.pose.position.y = right_y + y;
	msg.pose.position.z = right_z + z;
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
	pub_left = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_right = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	// capture and store initial positions of all models
	get_states();

	ROS_INFO("Initial Positions Captured, %f, %f, %f", head_x, right_x, left_x);
	
	spin();
}