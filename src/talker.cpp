/*
 *	talker.cpp
*/

#include <ros/ros.h>
#include <random>
#include <std_msgs/Float64.h>

int main(int argc, char**argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Float64>("/graph_y", 1);
	
	std_msgs::Float64 msg;
	
	std::random_device rnd;
	std::mt19937 engine(rnd());
	std::normal_distribution<double> dist(0.0, 1.0e-1);
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		msg.data = dist(engine);
		// std::cout << msg.data << std::endl;
		pub.publish(msg);
		loop_rate.sleep();
	}
}
