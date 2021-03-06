#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>

ros::Time time_now;
ros::Time time_last;
bool first_callback_odom = true;
bool inipose_is_available = false;
nav_msgs::Odometry odom_now;
nav_msgs::Odometry odom_last;
Eigen::MatrixXd Position;
tf::Quaternion q_pose;
tf::Quaternion q_slam_now;
tf::Quaternion q_slam_last;

void callback_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	// std::cout << "CALLBACK POSE" << std::endl;
	
	q_slam_now = tf::Quaternion(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
	q_slam_now.normalize();
	
	if(inipose_is_available){
		tf::Quaternion q_relative_rotation = q_slam_last.inverse()*q_slam_now;
		q_pose = q_pose*q_relative_rotation;
		q_pose.normalize();
		quaternionTFToMsg(q_pose, odom_now.pose.pose.orientation);
	}
	else	q_slam_last = q_slam_now;
	
	q_slam_last = q_slam_now;

	if(false)	odom_now.pose.pose.orientation = msg->pose.orientation;
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
	time_now = ros::Time::now();
	double dt = (time_now - time_last).toSec();
	time_last = time_now;
	
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
	transform.child_frame_id = "/lsd_odom";
	transform.transform.translation.x = odom_now.pose.pose.position.x;
	transform.transform.translation.y = odom_now.pose.pose.position.y;
	transform.transform.translation.z = odom_now.pose.pose.position.z;
	transform.transform.rotation = odom_now.pose.pose.orientation;
	broadcaster.sendTransform(transform);
}

void initialize_odom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/lsd_odom";
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
	ros::init(argc, argv, "lsd_odom");
	ros::NodeHandle nh;

	ros::Subscriber sub_odom = nh.subscribe("/odom", 1, callback_odom);
	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	ros::Subscriber sub_pose = nh.subscribe("/lsd_slam/pose", 1, callback_pose);
	ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/lsd_odom", 1);

	time_now = ros::Time::now();
	time_last = ros::Time::now();

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
