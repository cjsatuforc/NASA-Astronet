#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

using namespace ros;

Publisher pub;
ServiceClient client;

int main(int argc, char **argv) {
	init(argc, argv, "hbirddgState");
	NodeHandle nh;

	pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10000);
	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	// publish model states
	gazebo_msgs::ModelState hbirddg;

	// call services and get model states
	gazebo_msgs::GetModelState model1;
	model1.request.model_name = "/hbirddg";
	
	ROS_WARN("Starting Service for Publishing the State of Quad_hbirddg");

	while(ok()){
		if(client.call(model1)){
			hbirddg.model_name = "/hbirddg";
			hbirddg.pose.position.x = model1.response.pose.position.x;
			hbirddg.pose.position.y = model1.response.pose.position.y;
			hbirddg.pose.position.z = model1.response.pose.position.z;
			hbirddg.pose.orientation.x = model1.response.pose.orientation.x;
			hbirddg.pose.orientation.y = model1.response.pose.orientation.y;
			hbirddg.pose.orientation.z = model1.response.pose.orientation.z;
			hbirddg.pose.orientation.w = model1.response.pose.orientation.w;
			pub.publish(hbirddg);
		}
		else{
			ROS_ERROR("Failed to call service");
			break;
		}
	}
}