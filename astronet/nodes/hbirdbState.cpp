#include <ros/ros.h>
#include <stdlib.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <asctec_hl_comm/mav_ctrl.h>

using namespace ros;

Publisher pub;
Subscriber sub;
ServiceClient client;

void publish_hbirdb(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0, float yaw=0.0) {
	gazebo_msgs::GetModelState modelstate;
	modelstate.request.model_name = "/hbirdb";
	client.call(modelstate);

	// preserve position and orientation
	msg.pose.position.x = modelstate.response.pose.position.x;
	msg.pose.position.y = modelstate.response.pose.position.y;
	msg.pose.position.z = modelstate.response.pose.position.z;
	
	msg.pose.orientation.x = modelstate.response.pose.orientation.x;
	msg.pose.orientation.y = modelstate.response.pose.orientation.y;
	msg.pose.orientation.z = modelstate.response.pose.orientation.z;
	msg.pose.orientation.w = modelstate.response.pose.orientation.w;

	// apply linear and angular velocities
	msg.twist.linear.x = x;
	msg.twist.linear.y = y;
	msg.twist.linear.z = z;
	msg.twist.angular.x = 0;
	msg.twist.angular.y = 0;
	msg.twist.angular.z = yaw;
}


void listener_hbirdb(const asctec_hl_comm::mav_ctrl &msg) {
	ROS_WARN("HbirdB Linear Velocities:=\nx: %.2f, y: %.2f, z: %.2f",msg.x, msg.y, msg.z);
	gazebo_msgs::ModelState new_msg;
	new_msg.model_name = "/hbirdb";
	publish_hbirdb(new_msg, msg.x, msg.y, msg.z, msg.yaw);
}

int main(int argc, char **argv) {
	init(argc, argv, "hbirdbState");
	NodeHandle nh;

	pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10000);
	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	// publish model states
	gazebo_msgs::ModelState hbirdb;

	// call services and get model states
	gazebo_msgs::GetModelState model1;
	model1.request.model_name = "/hbirdb";
	
	ROS_WARN("Starting Service for Publishing the State of Quad_hbirdb");

	while(ok()){
		if(client.call(model1)){
			hbirdb.model_name = "/hbirdb";
			hbirdb.pose.position.x = model1.response.pose.position.x;
			hbirdb.pose.position.y = model1.response.pose.position.y;
			hbirdb.pose.position.z = model1.response.pose.position.z;
			hbirdb.pose.orientation.x = model1.response.pose.orientation.x;
			hbirdb.pose.orientation.y = model1.response.pose.orientation.y;
			hbirdb.pose.orientation.z = model1.response.pose.orientation.z;
			hbirdb.pose.orientation.w = model1.response.pose.orientation.w;
			pub.publish(hbirdb);
		}
		else{
			ROS_ERROR("Failed to call service");
			break;
		}
	}
}