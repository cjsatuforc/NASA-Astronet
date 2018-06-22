/*
Node to get current state of quadrotor
and/or astrobee in manual gesture mode 
Astrobee Name - HBIRDDG
*/

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Transform.h>

using namespace ros;

Publisher pub;
ServiceClient client;

int main(int argc, char **argv) {
	init(argc, argv, "hbirddgState");
	NodeHandle nh;

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	pub = nh.advertise<geometry_msgs::Transform>("hbirddgState", 10000);

	// publish model states
	geometry_msgs::Transform hbirddg;

	// call services and get model states
	gazebo_msgs::GetModelState model1;
	model1.request.model_name = "hbirddg";
	
	ROS_WARN("Starting Service for Publishing the State of Quad_hbirddg");

	while(ok()){
		if(client.call(model1)){
			hbirddg.translation.x = model1.response.pose.position.x;
			hbirddg.translation.y = model1.response.pose.position.y;
			hbirddg.translation.z = model1.response.pose.position.z;
			hbirddg.rotation.x = model1.response.pose.orientation.x;
			hbirddg.rotation.y = model1.response.pose.orientation.y;
			hbirddg.rotation.z = model1.response.pose.orientation.z;
			hbirddg.rotation.w = model1.response.pose.orientation.w;
			pub.publish(hbirddg);
		}
		else{
			ROS_WARN("Waiting for HBIRDDG to be detected");
		}
	}
}