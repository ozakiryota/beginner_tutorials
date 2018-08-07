#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

sensor_msgs::Imu imu;
// float LOOP_RATE;
std::string TARGET_FRAME;
ros::Time current_time;
ros::Time last_time;
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

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;
	// if(first_callback)	q = tf::Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	if(first_callback)	q = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
	else{
		double roll = -imu.angular_velocity.x*dt;
		double pitch = -imu.angular_velocity.y*dt;
		double yaw = -imu.angular_velocity.z*dt;
		q = tf::createQuaternionFromRPY(roll, pitch, yaw)*q;
	}

	const double g = 9.80665;
	v[0] += -imu.linear_acceleration.x*dt;
	v[1] += -imu.linear_acceleration.y*dt;
	v[2] += (imu.linear_acceleration.z - g)*dt;
	x[0] += v[0]*dt;
	x[1] += v[1]*dt;
	x[2] += v[2]*dt;

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x[0], x[1], x[2]));
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), TARGET_FRAME, "imu_"));

	first_callback = false;
}

void callback_imu1(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "callback_imu" << std::endl;
	imu = *msg;

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	// if(first_callback)	q = tf::Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	if(first_callback)	q = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
	else{	
	double roll;
	double pitch;
	double yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	Eigen::MatrixXf R(3, 1);
	R <<	roll,
	  		pitch,
			yaw;	
	double roll_ = -imu.angular_velocity.x*dt;
	double pitch_ = -imu.angular_velocity.y*dt;
	double yaw_ = -imu.angular_velocity.z*dt;
	Eigen::MatrixXf U(3, 1);
	U <<	roll_,
	  		pitch_,
			yaw_;
	Eigen::MatrixXf A(3, 3);
	A <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf B(3, 3);
	B <<	1,	sin(roll)*tan(pitch),	cos(roll)*tan(pitch),
	  		0,	cos(roll),	-sin(pitch),
			0,	sin(roll)/cos(pitch),	cos(roll)/cos(pitch);

	R = A*R + B*U;
	q = tf::createQuaternionFromRPY(R(0, 0), R(1, 0), R(2, 0));
	}

	const double g = 9.80665;
	v[0] += -imu.linear_acceleration.x*dt;
	v[1] += -imu.linear_acceleration.y*dt;
	v[2] += (imu.linear_acceleration.z - g)*dt;
	x[0] += v[0]*dt;
	x[1] += v[1]*dt;
	x[2] += v[2]*dt;

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x[0], x[1], x[2]));
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), TARGET_FRAME, "imu_"));

	first_callback = false;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_imu");
	ros::NodeHandle nh; 
	ros::NodeHandle local_nh("~");
	local_nh.getParam("TARGET_FRAME", TARGET_FRAME);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	// ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu1);

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	while(ros::ok()){
		ros::spinOnce();
	}
}
