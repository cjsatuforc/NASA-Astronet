#include <ros/ros.h>
#include <stdlib.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <asctec_hl_comm/mav_ctrl.h>

using namespace ros;

// make state publishers for each of the quads
Publisher pub_hbirdb;
Publisher pub_hbirddg;

// make subscribers for each of the quads
Subscriber sub_hbirdb;
Subscriber sub_hbirddg;

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


void publish_hbirddg(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0, float yaw=0.0) {
	gazebo_msgs::GetModelState modelstate;
	modelstate.request.model_name = "/hbirddg";
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
	// publish_hbirdb(new_msg, msg.x, msg.y, msg.z, msg.yaw);
}


void listener_hbirddg(const asctec_hl_comm::mav_ctrl &msg) {
	ROS_WARN("HbirdDG Linear Velocities:=\nx: %.2f, y: %.2f, z: %.2f",msg.x, msg.y, msg.z);
	gazebo_msgs::ModelState new_msg;
	new_msg.model_name = "/hbirddg";
	publish_hbirddg(new_msg, msg.x, msg.y, msg.z, msg.yaw);
}


int main(int argc, char **argv) {
	init(argc, argv, "publisher");
	NodeHandle nh;
	
	sub_hbirdb = nh.subscribe("/cov_ctrl_1", 10000, &listener_hbirdb);
	sub_hbirddg = nh.subscribe("/cov_ctrl_2", 10000, &listener_hbirddg);
	
	pub_hbirdb = nh.advertise<gazebo_msgs::ModelState>("/hbirdb/gazebo/set_model_state", 10000);
	pub_hbirddg = nh.advertise<gazebo_msgs::ModelState>("/hbirddg/gazebo/set_model_state", 10000);

	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	spin();
}