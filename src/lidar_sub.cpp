//lidar_sub.cpp

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan laser;

bool flag = true;
int count = 0;

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
	if(count<3)	laser = *msg;
	laser.header.frame_id = "test";
	count ++;
	flag = false;
	// std::cout << "laser_callback" << std::endl;
	// std::cout << laser.ranges.size() << std::endl;	//size=417
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lidar_sub");
	ros::NodeHandle nh;
	
	ros::Subscriber laser_sub = nh.subscribe("/sq_lidar/scan", 100, laser_callback);
	ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/test/laser",100);

	// laser.header.frame_id = "test";

	ros::Rate loop_rate(10);
	while(ros::ok()){
		// std::cout << "test" << std::endl;
		if(!laser.ranges.empty()){
			laser_pub.publish(laser);
		}
		ros::spinOnce();
		// loop_rate.sleep();
	}
}
