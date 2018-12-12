#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

bool inipose_is_available = false;
double ini_r = 0.0;
double ini_p = 0.0;
double ini_y = 0.0;

void callback_odom1(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "--- proposed ---" << std::endl;
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	std::cout << "position[m]: " << "(" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z << ")" << std::endl;
	std::cout << "Euclidian distance[m]: " << sqrt(msg->pose.pose.position.x*msg->pose.pose.position.x + msg->pose.pose.position.y*msg->pose.pose.position.y + msg->pose.pose.position.z*msg->pose.pose.position.z) << std::endl;
	std::cout << "pose[rad]: " << "(" << roll << ", " << pitch << ", " << yaw << ")  ";
	std::cout << "error[rad]: " << "(" << roll-ini_r << ", " << pitch-ini_p << ", " << yaw-ini_y << ")" << std::endl;
	roll = roll/M_PI*180.0;
	pitch = pitch/M_PI*180.0;
	yaw = yaw/M_PI*180.0;
	std::cout << "pose[deg]: " << "(" << roll << ", " << pitch << ", " << yaw << ")  ";
	std::cout << "error[deg]: " << "(" << roll-ini_r/M_PI*180.0 << ", " << pitch-ini_p/M_PI*180.0 << ", " << yaw-ini_y/M_PI*180.0 << ")" << std::endl;
}


void callback_odom2(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "--- lsdslam ---" << std::endl;
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	std::cout << "position[m]: " << "(" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z << ")" << std::endl;
	std::cout << "Euclidian distance[m]: " << sqrt(msg->pose.pose.position.x*msg->pose.pose.position.x + msg->pose.pose.position.y*msg->pose.pose.position.y + msg->pose.pose.position.z*msg->pose.pose.position.z) << std::endl;
	std::cout << "pose[rad]: " << "(" << roll << ", " << pitch << ", " << yaw << ")  ";
	std::cout << "error[rad]: " << "(" << roll-ini_r << ", " << pitch-ini_p << ", " << yaw-ini_y << ")" << std::endl;
	roll = roll/M_PI*180.0;
	pitch = pitch/M_PI*180.0;
	yaw = yaw/M_PI*180.0;
	std::cout << "pose[deg]: " << "(" << roll << ", " << pitch << ", " << yaw << ")  ";
	std::cout << "error[deg]: " << "(" << roll-ini_r/M_PI*180.0 << ", " << pitch-ini_p/M_PI*180.0 << ", " << yaw-ini_y/M_PI*180.0 << ")" << std::endl;
}

void callback_odom3(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "--- gyrodometry ---" << std::endl;
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	std::cout << "position[m]: " << "(" << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z << ")" << std::endl;
	std::cout << "Euclidian distance[m]: " << sqrt(msg->pose.pose.position.x*msg->pose.pose.position.x + msg->pose.pose.position.y*msg->pose.pose.position.y + msg->pose.pose.position.z*msg->pose.pose.position.z) << std::endl;
	std::cout << "pose[rad]: " << "(" << roll << ", " << pitch << ", " << yaw << ")  ";
	std::cout << "error[rad]: " << "(" << roll-ini_r << ", " << pitch-ini_p << ", " << yaw-ini_y << ")" << std::endl;
	roll = roll/M_PI*180.0;
	pitch = pitch/M_PI*180.0;
	yaw = yaw/M_PI*180.0;
	std::cout << "pose[deg]: " << "(" << roll << ", " << pitch << ", " << yaw << ")  ";
	std::cout << "error[deg]: " << "(" << roll-ini_r/M_PI*180.0 << ", " << pitch-ini_p/M_PI*180.0 << ", " << yaw-ini_y/M_PI*180.0 << ")" << std::endl;
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		tf::Quaternion q_ini;
		quaternionMsgToTF(*msg, q_ini);
		tf::Matrix3x3(q_ini).getRPY(ini_r, ini_p, ini_y);
		inipose_is_available = true;
	}
	
	static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/initial_pose";
	transform.transform.translation.x = 0.0;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = 0.0;
	transform.transform.rotation = *msg;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_printer");
	ros::NodeHandle nh;

	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	ros::Subscriber sub_odom1 = nh.subscribe("/odom3d_with_posemsg", 1, callback_odom1);
	ros::Subscriber sub_odom2 = nh.subscribe("/lsd_odom", 1, callback_odom2);
	ros::Subscriber sub_odom3 = nh.subscribe("/gyrodometry3d", 1, callback_odom3);
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		std::cout << "========================" << std::endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
}
