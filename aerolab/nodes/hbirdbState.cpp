/*
Node to get current state of quadrotor
and/or astrobee in coverage mode 
Astrobee Name - HBIRDB
*/

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Transform.h>

using namespace ros;

Publisher pub;
ServiceClient client;

int main(int argc, char **argv) {
	init(argc, argv, "hbirdbState");
	NodeHandle nh;

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	pub = nh.advertise<geometry_msgs::Transform>("hbirdbState", 10000);

	// publish model states
	geometry_msgs::Transform hbirdb;

	// call services and get model states
	gazebo_msgs::GetModelState model1;
	model1.request.model_name = "hbirdb";
	
	ROS_WARN("Starting Service for Publishing the State of Quad_hbirdb");

	while(ok()){
		if(client.call(model1)){
			hbirdb.translation.x = model1.response.pose.position.x;
			hbirdb.translation.y = model1.response.pose.position.y;
			hbirdb.translation.z = model1.response.pose.position.z;
			hbirdb.rotation.x = model1.response.pose.orientation.x;
			hbirdb.rotation.y = model1.response.pose.orientation.y;
			hbirdb.rotation.z = model1.response.pose.orientation.z;
			hbirdb.rotation.w = model1.response.pose.orientation.w;
			pub.publish(hbirdb);
		}
		else{
			ROS_WARN("Waiting for HBIRDB to be detected");
		}
	}
}