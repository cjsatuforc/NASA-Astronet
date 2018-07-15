#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

using namespace ros;

int ctr = 0;

int main(int argc, char **argv){
	init(argc, argv, "rand_velo_gen");
	NodeHandle nh;

	// create a publisher
	Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10000);

	srand(time(0));

	// publish at a rate of 2Hz
	Rate rate(1);

	float scale = 0.8;
	
	ROS_WARN_STREAM("Running node to generate random velocities");

	// while the node is not killed
	while (ok()){
		ctr++;
		geometry_msgs::Twist msg;
		if(ctr==5 || ctr==6){
			msg.linear.x = scale*2*double(rand())/double(RAND_MAX) - scale*1;
			msg.linear.y = 0.4;
			msg.linear.z = 0.3;
		}
		else if(ctr==9){
			msg.linear.x = -scale*2*double(rand())/double(RAND_MAX) - scale*1;
			msg.linear.y = -0.2;
			msg.linear.z = 0.5;	
		}
		else if(ctr==12){
			msg.linear.x = scale*2*double(rand())/double(RAND_MAX) - scale*1;
			msg.linear.y = -0.3;
			msg.linear.z = -0.4;	
		}
		else if(ctr==15){
			msg.linear.x = -scale*2*double(rand())/double(RAND_MAX) - scale*1;
			msg.linear.y = 0.6;
			msg.linear.z = -0.3;	
		}
		else if(ctr==18){
			msg.linear.x = -1;
			msg.linear.y = 0.1;
			msg.linear.z = -0.1;	
		}
		else if(ctr==22){
			msg.linear.x = -0.3;
			msg.linear.y = -scale*2*double(rand())/double(RAND_MAX) - scale*1;
			msg.linear.z = 0.2;	
		}
		else{
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
		}
		// msg.angular.x = scale*2*double(rand())/double(RAND_MAX) - scale*1 ;
		// msg.angular.y = scale*2*double(rand())/double(RAND_MAX) - scale*1 ;
		// msg.angular.z = scale*2*double(rand())/double(RAND_MAX) - scale*1 ;

		// ROS_INFO("linear: x:%f y:%f z:%f \n angular: x:%f y:%f z:%f",msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
		printf("counter : %d\n",ctr );
		// publish message
		pub.publish(msg);

		// wait till time for next iteration
		rate.sleep();
	}
	shutdown();
}