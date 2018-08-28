#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// std::string SUB_MSG;
// std::string PARENT_FRAME;
// std::string CHILD_FRAME;

void callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/lsd_slam/cameraframe";
	
	tf::TransformListener listener;
	tf::StampedTransform transform_;
	try {
		listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform_);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	transform.transform.translation.x = transform_.getOrigin().x();
	transform.transform.translation.y = transform_.getOrigin().y();
	transform.transform.translation.z = transform_.getOrigin().z();

	// transform.transform.translation.x = msg->pose.position.z;
	// transform.transform.translation.y = -msg->pose.position.x;
	// transform.transform.translation.z = -msg->pose.position.y;
	// transform.transform.translation.x = 0.0;
	// transform.transform.translation.y = 0.0;
	// transform.transform.translation.z = 0.0;

	tf::Quaternion q(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
	quaternionTFToMsg(q, transform.transform.rotation);

	static tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_lsdslam");
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

