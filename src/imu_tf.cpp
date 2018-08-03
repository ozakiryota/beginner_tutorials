#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

sensor_msgs::Imu imu;
const float LOOP_RATE = 400.1;
// ros::Time current_time;
// ros::Time last_time;
tf::Quaternion q_last(0.0, 0.0, 0.0, 1.0);
tf::Quaternion q_current;

void get_rpy(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
	tf::Quaternion q_(q.x, q.y, q.z, q.w);
	tf::Matrix3x3(q_).getRPY(roll, pitch, yaw);
}

void get_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q)
{
	tf::Quaternion q_ = tf::createQuaternionFromRPY(roll, pitch, yaw);
	quaternionTFToMsg(q_, q);
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "callback_imu" << std::endl;
	imu = *msg;	
	double roll = -imu.angular_velocity.x*(1/LOOP_RATE);
	double pitch = -imu.angular_velocity.y*(1/LOOP_RATE);
	double yaw = -imu.angular_velocity.z*(1/LOOP_RATE);
	q_current = tf::createQuaternionFromRPY(roll, pitch, yaw);
	q_current = q_current*q_last;
	q_last = q_current;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	transform.setRotation(q_current);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu", "base_link"));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_imu");
	ros::NodeHandle nh; 
	ros::NodeHandle local_nh("~");
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);

	while(ros::ok()){
		ros::spinOnce();
	}
}
