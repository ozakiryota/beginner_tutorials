#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

sensor_msgs::Imu imu;
float LOOP_RATE;
// ros::Time current_time;
// ros::Time last_time;
tf::Quaternion q(0.0, 0.0, 0.0, 1.0);
std::vector<double> v(3, 0.0);
std::vector<double> x(3, 0.0);
bool first_callback = true;

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
	
	if(first_callback)	q = tf::Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.z);

	double roll = -imu.angular_velocity.x*(1/LOOP_RATE);
	double pitch = -imu.angular_velocity.y*(1/LOOP_RATE);
	double yaw = -imu.angular_velocity.z*(1/LOOP_RATE);
	q = tf::createQuaternionFromRPY(roll, pitch, yaw)*q;

	const double g = 9.80665;
	v[0] += -imu.linear_acceleration.x*(1/LOOP_RATE);
	v[1] += -imu.linear_acceleration.y*(1/LOOP_RATE);
	v[2] += (imu.linear_acceleration.z - g)*(1/LOOP_RATE);
	x[0] += v[0]*(1/LOOP_RATE);
	x[1] += v[1]*(1/LOOP_RATE);
	x[2] += v[2]*(1/LOOP_RATE);

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x[0], x[1], x[2]));
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "imu_"));

	first_callback = false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_imu");
	ros::NodeHandle nh; 
	ros::NodeHandle local_nh("~");
	local_nh.getParam("LOOP_RATE", LOOP_RATE);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);

	while(ros::ok()){
		ros::spinOnce();
	}
}
