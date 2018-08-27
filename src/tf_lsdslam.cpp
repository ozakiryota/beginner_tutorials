#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

// std::string SUB_MSG;
// std::string PARENT_FRAME;
// std::string CHILD_FRAME;

void callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/lsd_slam/cameraframe";
	
	transform.transform.translation.x = msg->pose.position.z;
	transform.transform.translation.y = -msg->pose.position.x;
	transform.transform.translation.z = -msg->pose.position.y;
	// transform.transform.translation.x = 0.0;
	// transform.transform.translation.y = 0.0;
	// transform.transform.translation.z = 0.0;

	tf::Quaternion tf_raw(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Quaternion rot = tf::createQuaternionFromRPY(-M_PI/2.0, 0.0, -M_PI/2.0);
	quaternionTFToMsg(rot*tf_raw, transform.transform.rotation);

	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_from_posemsg");
	ros::NodeHandle nh;
	// ros::NodeHandle local_nh("~");
	// local_nh.getParam("SUB_MSG", SUB_MSG);
	// local_nh.getParam("PARENT_FRAME", PARENT_FRAME);
	// local_nh.getParam("CHILD_FRAME", CHILD_FRAME);
	
	ros::Subscriber sub_inipose = nh.subscribe("/lsd_slam/pose", 1, callback);

	while(ros::ok()){
		ros::spinOnce();
	}
}

