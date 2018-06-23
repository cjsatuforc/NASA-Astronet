/*
Node to visualize and move Oculus Position with 
Optitrack messages and publish it to move the 
stereocamera model and the image topic subscibed 
*/

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TransformStamped.h>
#define frequency 10
#define scaleX 0.5
#define scaleY 0.5
#define scaleZ 0.2
#define scaleAng 1

using namespace ros;

Subscriber sub_oculus;
Publisher pub_oculus;
ServiceClient client;

// declare initial values for the states of each model
float oculus_x = -3.6, oculus_y = 0, oculus_z = 5;
// origin of the oculus to be set from the first reading
float origin_x, origin_y, origin_z;
float origin_qx, origin_qy, origin_qz, origin_qw;

int ctr = 1;

// Model Publisher
void publish_oculus(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0, float qx=0.0, float qy=0.0, float qz=0.0, float qw=0.0) {
	
	// Set the initial position to the first second of adjusting positions of the oculus in the vicon
	if (ctr<60){
		origin_x = x;
		origin_y = y;
		origin_z = z;
		origin_qx = qx;
		origin_qy = qy;
		origin_qz = qz;
		origin_qw = qw;
	}
	// else publish every 5 frames to remove the jitter
	else if(ctr%frequency == 0) {
		msg.pose.position.x = oculus_x + scaleX*(x-origin_x);
		msg.pose.position.y = oculus_y + scaleY*(y-origin_y);
		msg.pose.position.z = oculus_z + scaleZ*(z-origin_z);
		msg.pose.orientation.x = scaleAng*(qx - origin_qx);
		msg.pose.orientation.y = scaleAng*(qy - origin_qy);
		msg.pose.orientation.z = scaleAng*(qz - origin_qz);
		msg.pose.orientation.w = scaleAng*(qw - origin_qw);
	}

	ctr++;
	
	pub_oculus.publish(msg);
}


// Vicon Data Listener
void listener_oculus(const geometry_msgs::TransformStamped &msg) {
	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;
	float z = msg.transform.translation.z;
	float qx = msg.transform.rotation.x;
	float qy = msg.transform.rotation.y;
	float qz = msg.transform.rotation.z;
	float qw = msg.transform.rotation.w;

	gazebo_msgs::ModelState oculus_msg;
	oculus_msg.model_name = "stereocamera";
	// ROS_INFO("Oculus Position: (%.2f, %.2f, %.2f), [%.2f, %.2f, %.2f, %.2f]", x,y,z, qx, qy, qz, qw);
	publish_oculus(oculus_msg, x, y, z, qx, qy, qz, qw);
}


// Main loop to orchestrate publishers and subscribers
int main(int argc, char **argv) {
	init(argc, argv, "oculusViz");
	NodeHandle nh;

	sub_oculus = nh.subscribe("/vicon/Oculus/Oculus", 10000, &listener_oculus);
	pub_oculus = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	ROS_WARN("Running Oculus Visualization Node");
	
	spin();
}