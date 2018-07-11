//imu_test.cpp

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

sensor_msgs::Imu imu;
geometry_msgs::PoseStamped imu00;

void imu_callback(const sensor_msgs::ImuPtr& msg)
{
	imu = *msg;
	std::cout << "imu_callback" << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	
	ros::Subscriber imu_sub = nh.subscribe("/imu/data", 100, imu_callback);
	ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data0", 100);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/imu/data00", 100);
	std::cout << "test" << std::endl;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		imu.linear_acceleration.x = 1.0;
		imu.linear_acceleration.y = 1.0;
		imu.linear_acceleration.z = 1.0;
		imu.angular_velocity.x = 1.0;
		imu.angular_velocity.y = 1.0;
		imu.angular_velocity.z = 1.0;
		imu_pub.publish(imu);

		imu00.header.frame_id = "/imu00";
		imu00.pose.position.x = 0;
		imu00.pose.position.y = 0;
		imu00.pose.position.z = 0;
		imu00.pose.orientation = imu.orientation;

		ros::spinOnce();
		loop_rate.sleep();
	}
}
