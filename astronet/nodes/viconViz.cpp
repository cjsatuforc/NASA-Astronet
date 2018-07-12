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

#define scaleX 1
#define scaleY 1
#define scaleZ 1
#define scaleAng 1
#define shoulderHeight 1.75
#define counterToStart 1000

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
float head_x = -0.75, head_y = 0, head_z = 5.25;
float torso_x = -0.75, torso_y = 0, torso_z = 4.75;
float left_x = -0.73, left_y = 0.25, left_z = 5;
float right_x = -0.73, right_y = -0.25, right_z = 5;

float origin_x, origin_y, origin_z;
float origin_qx, origin_qy, origin_qz, origin_qw;

int ctr = 0;

// Model Publishers
void publish_head(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0, float qx=0.0, float qy=0.0, float qz=0.0, float qw=0.0) {
	if (ctr<counterToStart) {
		origin_x = x;
		origin_y = y;
		origin_z = z;
		origin_qx = qx;
		origin_qy = qy;
		origin_qz = qz;
		origin_qw = qw;
	}
	else if(ctr==counterToStart) {
		ROS_WARN("Starting to track Head Positions");
	}
	else {
		// Normalize Quaternions
		msg.pose.position.x = head_x + scaleX*(x-origin_x);
		msg.pose.position.y = head_y + scaleY*(y-origin_y);
		msg.pose.position.z = head_z + scaleZ*(z-origin_z);
		msg.pose.orientation.x = scaleAng*(qx);
		msg.pose.orientation.y = scaleAng*(qy);
		msg.pose.orientation.z = scaleAng*(qz);
		msg.pose.orientation.w = scaleAng*(qw);
	}

	pub_head.publish(msg);
	ctr++;
	
}

void publish_torso(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	// preserve position and orientation
	if (ctr>counterToStart) {
		msg.pose.position.x = torso_x + scaleX*(x-origin_x);
		msg.pose.position.y = torso_y + scaleY*(y-origin_y);
		msg.pose.position.z = torso_z + scaleZ*(z-origin_z); // setting average value of head_z as origin
		pub_torso.publish(msg);
	}
}


void publish_left(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	if (ctr>counterToStart) {
		// preserve position and orientation
		msg.pose.position.x = left_x + scaleX*(x-origin_x);
		msg.pose.position.y = left_y + scaleY*(y-origin_y);
		msg.pose.position.z = left_z + scaleZ*(z-origin_z);
		pub_left.publish(msg);
	}
}


void publish_right(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0) {
	if (ctr>counterToStart) {
		// preserve position and orientation
		msg.pose.position.x = right_x + scaleX*(x-origin_x);
		msg.pose.position.y = right_y + scaleY*(y-origin_y);
		msg.pose.position.z = right_z + scaleZ*(z-origin_z);
		pub_right.publish(msg);
	}
}


// Vicon Data Listeners
void listener_head(const geometry_msgs::TransformStamped &msg) {
	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;
	float z = msg.transform.translation.z;
	float qx = msg.transform.rotation.x;
	float qy = msg.transform.rotation.y;
	float qz = msg.transform.rotation.z;
	float qw = msg.transform.rotation.w;

	// Round Decimals and Normalize
	x = round( x * 1000.0 ) / 1000.0;
	y = round( y * 1000.0 ) / 1000.0;
	z = round( z * 1000.0 ) / 1000.0;
	qx = round( qx * 100.0 ) / 100.0;
	qy = round( qy * 100.0 ) / 100.0;
	qz = round( qz * 100.0 ) / 100.0;
	qw = round( qw * 100.0 ) / 100.0;
	
	float normQ = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
	qx /= normQ ; qy /= normQ; qz /= normQ; qw /= normQ;

	gazebo_msgs::ModelState head_msg;
	gazebo_msgs::ModelState torso_msg;
	head_msg.model_name = "head";
	torso_msg.model_name = "torso";
	
	publish_head(head_msg, x, y, z, qx, qy, qz, qw);
	publish_torso(torso_msg, x, y, z);
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

	sub_head = nh.subscribe("/vicon/Oculus/Oculus", 10000, &listener_head);
	sub_left = nh.subscribe("/vicon/Left_Arm/Left_Arm", 10000, &listener_left);
	sub_right = nh.subscribe("/vicon/Right_Arm/Right_Arm", 10000, &listener_right);

	pub_head = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_torso = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_left = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	pub_right = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	spin();
}