#include <ros/ros.h>
#include <stdlib.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Twist.h>

using namespace ros;

Publisher pub;
Subscriber sub_vel;
ServiceClient client;

// method to integrate velocities and publish poses
void publish_pose(gazebo_msgs::ModelState &msg, float x=0.0, float y=0.0, float z=0.0, float qx=0.0, float qy=0.0, float qz=0.0) {
	// Get the current position of the astrobee
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
	msg.twist.angular.x = qx;
	msg.twist.angular.y = qy;
	msg.twist.angular.z = qz;

	ROS_INFO("\nPublishing Velocities := \nlinear- x:%f, y:%f, z:%f \nangular- x:%f, y:%f, z:%f\n",msg.twist.linear.x,msg.twist.linear.y,msg.twist.linear.z,msg.twist.angular.x,msg.twist.angular.y,msg.twist.angular.z);

	// publish
	pub.publish(msg);
}

void listener_vel(const geometry_msgs::Twist &msg) {
	gazebo_msgs::ModelState new_msg;
	new_msg.model_name = "/hbirdb";
	publish_pose(new_msg, msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
}


int main(int argc, char **argv){
	init(argc, argv, "publish");
	
	NodeHandle nh;

	pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10000);
	sub_vel = nh.subscribe("turtle1/cmd_vel", 10000, &listener_vel);
	client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	
	spin();
}