/*
Node to publish control messages to 
quadrotor/ astrobee in coverage mode 
Astrobee Name - HBIRDB
*/

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <asctec_hl_comm/mav_ctrl.h>

using namespace ros;

Publisher pub;
Subscriber sub;
ServiceClient client;

void publish_hbirdb(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0, float yaw=0.0) {
	gazebo_msgs::GetModelState modelstate;
	modelstate.request.model_name = "hbirdb";
	if(client.call(modelstate)){
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

		ROS_WARN("Publishing Calculated States for HbirdB");

		pub.publish(msg);
	}
	else {
		ROS_WARN("Waiting for HBIRDB to be detected");
	}
}

void listener_hbirdb(const asctec_hl_comm::mav_ctrl &msg) {
	ROS_WARN("HbirdB:= x: %.2f, y: %.2f, z: %.2f, yaw: %.2f",msg.x, msg.y, msg.z, msg.yaw);
	gazebo_msgs::ModelState new_msg;
	new_msg.model_name = "hbirdb";
	publish_hbirdb(new_msg, msg.x, msg.y, msg.z, msg.yaw);
}


int main(int argc, char **argv) {
	init(argc, argv, "hbirdbPub");
	NodeHandle nh;
	
	sub = nh.subscribe("/cov_ctrl_1", 10000, &listener_hbirdb);
	pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10000);
	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	spin();
}