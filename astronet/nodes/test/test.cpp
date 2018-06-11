#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <stdlib.h>

using namespace ros;

ServiceClient client;

int main(int argc, char **argv){
	init(argc, argv, "getState");
	NodeHandle nh;
	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gazebo_msgs::GetModelState modelstate;
	modelstate.request.model_name = "/";
	while(ok()){
		if(client.call(modelstate)){
			ROS_INFO_STREAM("x: "<<modelstate.response.pose.position.x<<" y: "<<modelstate.response.pose.position.y<<" z: "<<modelstate.response.pose.position.z);
		}
		else{
			ROS_ERROR("Failed to call service");
			break;
		}	
	}
	
	spin();
}
