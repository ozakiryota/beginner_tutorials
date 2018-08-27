#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

std::string SUB_MSG;
std::string PARENT_FRAME;
std::string CHILD_FRAME;

void callback(const geometry_msgs::PoseConstPtr& msg)
{
	static tf::TransformBroadcaster broadcaster;
	geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = PARENT_FRAME;
	transform.child_frame_id = CHILD_FRAME;
	transform.transform.translation.x = msg->position.x;
	transform.transform.translation.y = msg->position.y;
	transform.transform.translation.z = msg->position.z;
	transform.transform.rotation = msg->orientation;
	broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_from_posemsg");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	local_nh.getParam("SUB_MSG", SUB_MSG);
	local_nh.getParam("PARENT_FRAME", PARENT_FRAME);
	local_nh.getParam("CHILD_FRAME", CHILD_FRAME);
	
	ros::Subscriber sub_inipose = nh.subscribe(SUB_MSG, 1, callback);

	while(ros::ok()){
		ros::spinOnce();
	}
}

