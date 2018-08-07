#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>

sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
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
	// current_time = ros::Time::now();
	// double dt = (current_time - last_time).toSec();
	// last_time = current_time;
	// std::cout << "current_time.toSec() = " << current_time.toSec() << std::endl;
	// std::cout << "last_time.toSec() = " << last_time.toSec() << std::endl;
	// std::cout << "dt = " << dt << std::endl;
	
	// double roll;
	// double pitch;
	// double yaw;
	// get_rpy(odom.pose.pose.orientation, roll, pitch, yaw);
	// roll += imu.angular_velocity.x*(1/LOOP_RATE);
	// pitch += imu.angular_velocity.y*(1/LOOP_RATE);
	// yaw += imu.angular_velocity.z*(1/LOOP_RATE);
	// roll += imu.angular_velocity.x*dt;
	// pitch += imu.angular_velocity.y*dt;
	// yaw += imu.angular_velocity.z*dt;

	// get_quaternion(roll, pitch, yaw, odom.pose.pose.orientation);
	// Eigen::MatrixXf R(3, 3);
	// R <<	cos(pitch)*cos(roll),	sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll),	sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll),
	// 		cos(pitch)*sin(roll),	sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll),	-sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll),
	// 		-sin(pitch),	sin(yaw)*cos(pitch),	cos(yaw)*cos(pitch);
	//
	// double roll_;
	// double pitch_;
	// double yaw_;
	// get_rpy(odom.pose.pose.orientation, roll_, pitch_, yaw_);
	// Eigen::MatrixXf X_(3, 1);
	// X_ <<	0.0,
	//    		0.0,
	// 		0.0;
    //
	// Eigen::MatrixXf X(3, 1);
	// X = R*X_;
    //
	// std::cout << "roll_ = " << roll_ << std::endl;
	// std::cout << "X(0, 0) = " << X(0, 0) << std::endl;
    //
	// get_quaternion(X(0,0), X(1,0), X(2,0), odom.pose.pose.orientation);

	
	double roll = imu.angular_velocity.x*(1/LOOP_RATE);
	double pitch = imu.angular_velocity.y*(1/LOOP_RATE);
	double yaw = imu.angular_velocity.z*(1/LOOP_RATE);
	// get_quaternion(roll, pitch, yaw, odom.pose.pose.orientation);
	// q_current = tf::createQuaternionFromRPY(roll, pitch, yaw);
	q_current = tf::createQuaternionFromRPY(roll, pitch, yaw);
	// q_current = q_current*q_last;
	q_current = q_current*q_last;
	q_last = q_current;

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	// tf::Quaternion q;
	// q.setRPY(roll, pitch, yaw);
	transform.setRotation(q_current);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu", "base_link"));
}

void prediction(void)
{
	double roll;
	double pitch;
	double yaw;
	tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
}

void initialize_odom(void)
{
	std::cout << "imu_tf" << std::endl;
	odom.header.frame_id = "/imu";
	odom.child_frame_id = "/base_link";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_estimation_imu");
	ros::NodeHandle nh; 
	ros::NodeHandle local_nh("~");
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);

	initialize_odom();
	// current_time = ros::Time::now();
	// last_time = ros::Time::now();
	
	// ros::Rate rate(LOOP_RATE);
	while(ros::ok()){
		ros::spinOnce();
		// rate.sleep();
	}
}
