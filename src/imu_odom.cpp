#include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
// #include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

geometry_msgs::Quaternion ini_pose;
sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
bool inipose_is_available = false;
Eigen::MatrixXf V(3, 1);
std::string TARGET_FRAME;
ros::Time current_time;
ros::Time last_time;

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "callback_imu" << std::endl;
	imu = *msg;

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;
	
	if(inipose_is_available){
		tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		double roll = -imu.angular_velocity.x*dt;
		double pitch = -imu.angular_velocity.y*dt;
		double yaw = -imu.angular_velocity.z*dt;
		q = tf::createQuaternionFromRPY(roll, pitch, yaw)*q;
		quaternionTFToMsg(q, odom.pose.pose.orientation);
		
		Eigen::MatrixXf Acc(3, 1);
		Acc <<	imu.linear_acceleration.x,
				imu.linear_acceleration.y,
				imu.linear_acceleration.z;
		Eigen::MatrixXf G(3, 1);
		G <<	0.0,
				0.0,
				9.80665;
		V = V + (ACC - G)*dt;
		odom.pose.pose.position.x += V(0, 0)*dt;
		odom.pose.pose.position.y += V(1, 0)*dt;
		odom.pose.pose.position.z += V(2, 0)*dt;
	}
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	ini_pose = *msg;
	if(!inipose_is_available){
		odom.header.frame_id = TARGET_FRAME;
		odom.child_frame_id = "/imu_";
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = ini_pose;
	}
	inipose_is_available = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_odom");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	local_nh.getParam("TARGET_FRAME", TARGET_FRAME);
	
	ros::Subscriber sub_inipose = nh.subscribe("/ini_pose", 10, callback_inipose);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/imu_odom", 1);

	current_time = ros::Time::now();
	last_time = ros::Time::now();
	V <<	0.0,
			0.0,
			0.0;

	ros::Rate loop_rate(10);
	while(ros::ok()){
		odom.header.stamp = ros::Time::now();
		if(inipose_is_available)	pub_odom.publish(odom);
		ros::spinOnce();
		loop_rate.sleep();
	}
}