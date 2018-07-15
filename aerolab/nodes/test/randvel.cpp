#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

using namespace ros;

int main(int argc, char **argv){
	init(argc, argv, "rand_velo_gen");
	NodeHandle nh;

	// create a publisher
	Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10000);

	srand(time(0));

	// publish at a rate of 2Hz
	Rate rate(60);

	float scale = 0.2;
	
	ROS_WARN_STREAM("Running node to generate random velocities");

	// while the node is not killed
	while (ok()){
		geometry_msgs::Twist msg;
		msg.linear.x = scale*2*double(rand())/double(RAND_MAX) - scale*1;
		msg.linear.y = scale*2*double(rand())/double(RAND_MAX) - scale*1;
		msg.linear.z = scale*2*double(rand())/double(RAND_MAX) - scale*1;
		msg.angular.x = scale*2*double(rand())/double(RAND_MAX) - scale*1 ;
		msg.angular.y = scale*2*double(rand())/double(RAND_MAX) - scale*1 ;
		msg.angular.z = scale*2*double(rand())/double(RAND_MAX) - scale*1 ;

		ROS_INFO("linear: x:%f y:%f z:%f \n angular: x:%f y:%f z:%f",msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);

		// publish message
		pub.publish(msg);

		// wait till time for next iteration
		rate.sleep();
	}
	shutdown();
}