#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/LU>

ros::Time time_now;
ros::Time time_last;
nav_msgs::Odometry odom2d_now;
nav_msgs::Odometry odom2d_last;
Eigen::MatrixXd Odom3d;
nav_msgs::Odometry odom3d_now;
nav_msgs::Odometry odom3d_last;
bool first_callback_odom = true;
bool first_callback_imu = true;
sensor_msgs::Imu bias;
bool bias_is_available = false;
bool inipose_is_available = false;
sensor_msgs::Imu imu_lowpass;

sensor_msgs::Imu imu_last;

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "CALLBACK IMU" << std::endl;
	time_now = ros::Time::now();
	double dt = (time_now - time_last).toSec();
	time_last = time_now;
	// std::cout << "dt_" << dt << std::endl;
	if(first_callback_imu){
		dt = 0.0;
		imu_lowpass = *msg;
	}

	const double ratio_lowpass = 1.0;
	imu_lowpass.angular_velocity.x = ratio_lowpass*msg->angular_velocity.x + (1.0 - ratio_lowpass)*imu_lowpass.angular_velocity.x;
	imu_lowpass.angular_velocity.y = ratio_lowpass*msg->angular_velocity.y + (1.0 - ratio_lowpass)*imu_lowpass.angular_velocity.y;
	imu_lowpass.angular_velocity.z = ratio_lowpass*msg->angular_velocity.z + (1.0 - ratio_lowpass)*imu_lowpass.angular_velocity.z;

	tf::Quaternion q(odom3d_now.pose.pose.orientation.x, odom3d_now.pose.pose.orientation.y, odom3d_now.pose.pose.orientation.z, odom3d_now.pose.pose.orientation.w);
	double wx = msg->angular_velocity.x*dt;
	double wy = msg->angular_velocity.y*dt;
	double wz = msg->angular_velocity.z*dt;
	wx = imu_lowpass.angular_velocity.x*dt;
	wy = imu_lowpass.angular_velocity.y*dt;
	wz = imu_lowpass.angular_velocity.z*dt;

	if(bias_is_available){
		wx -= bias.angular_velocity.x*dt;
		wy -= bias.angular_velocity.y*dt;
		wz -= bias.angular_velocity.z*dt;
	}
	else{
		wx = 0.0;
		wy = 0.0;
		wz = 0.0;
	}

	if(true){
		if(first_callback_imu)	imu_last = *msg;
		
		if(bias_is_available){
			wx = (msg->angular_velocity.x + imu_last.angular_velocity.x - 2.0*bias.angular_velocity.x)*dt/2.0;
			wy = (msg->angular_velocity.y + imu_last.angular_velocity.y - 2.0*bias.angular_velocity.y)*dt/2.0;
			wz = (msg->angular_velocity.z + imu_last.angular_velocity.z - 2.0*bias.angular_velocity.z)*dt/2.0;
		}
		else{
			wx = 0.0;
			wy = 0.0;
			wz = 0.0;
		}
		imu_last = *msg;
	}
	
	// q = q*tf::createQuaternionFromRPY(wx, wy, wz);
	q = tf::createQuaternionFromRPY(wx, wy, wz)*q;
	q.normalize();
	quaternionTFToMsg(q, odom3d_now.pose.pose.orientation);

	/*for comparison*/
	// odom3d_now.pose.pose.orientation = msg->orientation;

	first_callback_imu = false;
}

void callback_bias(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "CALLBACK BIAS" << std::endl;
	bias = *msg;
	bias_is_available = true;
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		odom3d_now.pose.pose.orientation = *msg;
		inipose_is_available = true;
		std::cout << "inipose_is_available = " << inipose_is_available << std::endl;
		std::cout << "initial pose = " << std::endl << *msg << std::endl;
	}   
}

Eigen::MatrixXd frame_rotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local)
{
	Eigen::MatrixXd Rot(3, 3); 
	Rot <<  q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,  2*(q.x*q.y + q.w*q.z),  2*(q.x*q.z - q.w*q.y),
			2*(q.x*q.y - q.w*q.z),  q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,  2*(q.y*q.z + q.w*q.x),
			2*(q.x*q.z + q.w*q.y),  2*(q.y*q.z - q.w*q.x),  q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	// std::cout << "X = " << std::endl << X << std::endl;
	if(from_global_to_local)    return Rot*X;
	else    return Rot.inverse()*X;
}

void callback_odom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	if(first_callback_odom)	odom2d_last = *msg;
	odom2d_now = *msg;
	Eigen::MatrixXd Global2d(3, 1);
	Global2d	<<	odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
					odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
					odom2d_now.pose.pose.position.z - odom2d_last.pose.pose.position.z;
	Eigen::MatrixXd Local2d = frame_rotation(odom2d_last.pose.pose.orientation, Global2d, true);
	Eigen::MatrixXd Global3d = frame_rotation(odom3d_last.pose.pose.orientation, Local2d, false);
	Odom3d = Odom3d + Global3d;
	odom3d_now.pose.pose.position.x = Odom3d(0, 0);
	odom3d_now.pose.pose.position.y = Odom3d(1, 0);
	odom3d_now.pose.pose.position.z = Odom3d(2, 0);
	
	// std::cout << "Local2d = " << std::endl << Local2d << std::endl;
	// std::cout << "Global3d = " << std::endl << Global3d << std::endl;

	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	
	first_callback_odom = false;
}

void broadcast_tf(void)
{
	static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/odom3d";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

void initialize_odom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/odom3d";
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
	ros::init(argc, argv, "odom2dto3d");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("/odom", 1, callback_odom);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1, callback_imu);
	ros::Subscriber sub_bias = nh.subscribe("/imu_bias", 1, callback_bias);
	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom3d", 1);

	time_now = ros::Time::now();
	time_last = ros::Time::now();

	Odom3d = Eigen::MatrixXd::Constant(3, 1, 0.0);
	initialize_odom(odom3d_last);
	initialize_odom(odom3d_now);

	ros::Rate loop_rate(500);
	while(ros::ok()){
		ros::spinOnce();
		pub.publish(odom3d_now);
		broadcast_tf();

		loop_rate.sleep();
	}
}
