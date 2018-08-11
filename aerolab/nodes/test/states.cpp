#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>

using namespace ros;

Publisher pub;
ServiceClient client;

int main(int argc, char **argv) {
	init(argc, argv, "getState");
	NodeHandle nh;

	pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10000);
	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	// publish model states
	gazebo_msgs::ModelState hbirdb;
	gazebo_msgs::ModelState hbirddg;

	// call services and get model states
	gazebo_msgs::GetModelState model1;
	gazebo_msgs::GetModelState model2;
	model1.request.model_name = "/hbirdb";
	model2.request.model_name = "/hbirddg";
	
	while(ok()){
		if(client.call(model1) && client.call(model2)){
			// ROS_WARN("Model 1 - HDBIRDB");
			// ROS_INFO_STREAM("x: "<<model1.response.pose.position.x<<" y: "<<model1.response.pose.position.y<<" z: "<<model1.response.pose.position.z);
			hbirdb.model_name = "/hbirdb";
			hbirdb.pose.position.x = model1.response.pose.position.x;
			hbirdb.pose.position.y = model1.response.pose.position.y;
			hbirdb.pose.position.z = model1.response.pose.position.z;
			hbirdb.pose.orientation.x = model1.response.pose.orientation.x;
			hbirdb.pose.orientation.y = model1.response.pose.orientation.y;
			hbirdb.pose.orientation.z = model1.response.pose.orientation.z;
			hbirdb.pose.orientation.w = model1.response.pose.orientation.w;
			pub.publish(hbirdb);

			// ROS_WARN("Model 2 - HDBIRDDG");
			// ROS_INFO_STREAM("x: "<<model2.response.pose.position.x<<" y: "<<model2.response.pose.position.y<<" z: "<<model2.response.pose.position.z);
			hbirddg.model_name = "/hbirddg";
			hbirddg.pose.position.x = model2.response.pose.position.x;
			hbirddg.pose.position.y = model2.response.pose.position.y;
			hbirddg.pose.position.z = model2.response.pose.position.z;
			hbirddg.pose.orientation.x = model2.response.pose.orientation.x;
			hbirddg.pose.orientation.y = model2.response.pose.orientation.y;
			hbirddg.pose.orientation.z = model2.response.pose.orientation.z;
			hbirddg.pose.orientation.w = model2.response.pose.orientation.w;
			pub.publish(hbirddg);
		}
		else{
			ROS_ERROR("Failed to call service");
			break;
		}
	}
}