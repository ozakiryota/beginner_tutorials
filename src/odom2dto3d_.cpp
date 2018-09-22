#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
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

void callback_pose(const geometry_msgs::PoseConstPtr& msg)
{
	// std::cout << "CALLBACK POSE" << std::endl;
	odom3d_now.pose.pose.orientation = msg->orientation;

	// tf::Quaternion q_test(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	// double r, p, y;
	// tf::Matrix3x3(q_test).getRPY(r, p, y);
	// q_test = tf::createQuaternionFromRPY(0.0, 0.0, y);
	// quaternionTFToMsg(q_test, odom3d_now.pose.pose.orientation);
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
	transform.child_frame_id = "/odom3d_";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

void initialize_odom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/odom3d_";
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
	ros::init(argc, argv, "odom2dto3d_");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("/odom", 1, callback_odom);
	ros::Subscriber sub_pose = nh.subscribe("/pose_imu_slam_walls", 1, callback_pose);
	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom3d_", 1);

	time_now = ros::Time::now();
	time_last = ros::Time::now();

	Odom3d = Eigen::MatrixXd::Constant(3, 1, 0.0);
	initialize_odom(odom3d_last);
	initialize_odom(odom3d_now);

	while(ros::ok()){
		ros::spinOnce();
		pub.publish(odom3d_now);
		broadcast_tf();
	}
}
