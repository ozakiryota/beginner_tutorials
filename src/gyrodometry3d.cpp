#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>

ros::Time time_now_odom;
ros::Time time_last_odom;
ros::Time time_now_imu;
ros::Time time_last_imu;
bool first_callback_odom = true;
bool inipose_is_available = false;
bool bias_is_available = false;
nav_msgs::Odometry odom_now;
nav_msgs::Odometry odom_last;
Eigen::MatrixXd Position;
tf::Quaternion q_pose;
sensor_msgs::Imu bias;

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "CALLBACK IMU" << std::endl;
	
	time_now_imu = ros::Time::now();
	double dt = (time_now_imu - time_last_imu).toSec();
	time_last_imu = time_now_imu;
	
	if(inipose_is_available){
		double delta_r = msg->angular_velocity.x*dt;
		double delta_p = msg->angular_velocity.y*dt;
		double delta_y = msg->angular_velocity.z*dt;
		if(bias_is_available){
			delta_r -= bias.angular_velocity.x*dt;
			delta_p -= bias.angular_velocity.y*dt;
			delta_y -= bias.angular_velocity.z*dt;
		}

		tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
		q_pose = q_pose*q_relative_rotation;
		q_pose.normalize();
		quaternionTFToMsg(q_pose, odom_now.pose.pose.orientation);
	}
	
	if(false)	odom_now.pose.pose.orientation = msg->orientation;
}

void callback_bias(const sensor_msgs::ImuConstPtr& msg)
{
	bias = *msg;
	bias_is_available = true;
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
	time_now_odom = ros::Time::now();
	double dt = (time_now_odom - time_last_odom).toSec();
	time_last_odom = time_now_odom;

	odom_now.twist = msg->twist;

	if(first_callback_odom){
		dt = 0.0;
		odom_last = odom_now;
	}
	
	Eigen::MatrixXd LocalVel(3, 1);
	LocalVel <<	odom_last.twist.twist.linear.x,
		  		odom_last.twist.twist.linear.y,
				odom_last.twist.twist.linear.z;
	Eigen::MatrixXd GlobalVel = frame_rotation(odom_last.pose.pose.orientation, LocalVel, false);
	Position = Position + GlobalVel*dt;

	odom_now.pose.pose.position.x = Position(0, 0);
	odom_now.pose.pose.position.y = Position(1, 0);
	odom_now.pose.pose.position.z = Position(2, 0);

	odom_last = odom_now;

	first_callback_odom = false;
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		quaternionMsgToTF(*msg, q_pose);
		inipose_is_available = true;
	}   
}

void broadcast_tf(void)
{
	static tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/gyrodometry3d";
	transform.transform.translation.x = odom_now.pose.pose.position.x;
	transform.transform.translation.y = odom_now.pose.pose.position.y;
	transform.transform.translation.z = odom_now.pose.pose.position.z;
	transform.transform.rotation = odom_now.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

void initialize_odom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/gyrodometry3d";
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
	ros::init(argc, argv, "gyrodometry3d");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("/odom", 1, callback_odom);
	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1, callback_imu);
	ros::Subscriber sub_bias = nh.subscribe("/imu_bias", 1, callback_bias);
	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/gyrodometry3d", 1);

	time_now_odom = ros::Time::now();
	time_last_odom = ros::Time::now();
	time_now_imu = ros::Time::now();
	time_last_imu = ros::Time::now();

	/*initialization*/
	Position = Eigen::MatrixXd::Constant(3, 1, 0.0);
	initialize_odom(odom_last);
	initialize_odom(odom_now);

	ros::Rate loop_rate(100);
	while(ros::ok()){
		ros::spinOnce();
		pub.publish(odom_now);
		broadcast_tf();

		loop_rate.sleep();
	}
}
