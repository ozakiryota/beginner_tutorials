#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class ImuOdometry{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_imu;
		/*publish*/
		ros::Publisher pub_odom;
		// tf::TransformBroadcaster tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry odom;
		/*objects*/
		tf::Quaternion q_pose{0.0, 0.0, 0.0, 1.0};
		sensor_msgs::Imu imu_last;
		/*time*/
		ros::Time time_now_imu;
		ros::Time time_last_imu;
		/*flags*/
		bool first_callback = true;
	public:
		ImuOdometry();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void Publication(void);
};

ImuOdometry::ImuOdometry()
{
	sub_imu = nh.subscribe("/imu/data", 1, &ImuOdometry::CallbackIMU, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/imu_odometry", 1);
	InitializeOdom(odom);
}

void ImuOdometry::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/imu_odometry";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.linear.z = 0.0;
}

void ImuOdometry::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	time_now_imu = ros::Time::now();
	double dt;
	try{
		dt = (time_now_imu - time_last_imu).toSec();
	}
	catch(std::runtime_error& ex){
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_last_imu = time_now_imu;
	if(first_callback){
		dt = 0.0;
		imu_last = *msg;
		quaternionMsgToTF(msg->orientation, q_pose);
	}
	else{
		/*position*/
		const double g = -9.80665;	//[m/s^2]
		tf::Quaternion q_local_acc(
				-msg->linear_acceleration.x,
				-msg->linear_acceleration.y,
				-msg->linear_acceleration.z,
				0.0);
		tf::Quaternion q_global_acc = q_pose.inverse()*q_local_acc*q_pose;
		// tf::Quaternion q_global_acc = q_pose*q_local_acc*q_pose.inverse();
		odom.twist.twist.linear.x += q_global_acc.x()*dt;
		odom.twist.twist.linear.y += q_global_acc.y()*dt;
		odom.twist.twist.linear.z += (q_global_acc.z() - g)*dt;
		odom.pose.pose.position.x += odom.twist.twist.linear.x*dt;
		odom.pose.pose.position.y += odom.twist.twist.linear.y*dt;
		odom.pose.pose.position.z += odom.twist.twist.linear.z*dt;

		/*pose*/
		double delta_r = (msg->angular_velocity.x + imu_last.angular_velocity.x)*dt/2.0;
		double delta_p = (msg->angular_velocity.y + imu_last.angular_velocity.y)*dt/2.0;
		double delta_y = (msg->angular_velocity.z + imu_last.angular_velocity.z)*dt/2.0;
		// if(bias_is_available){
		// 	delta_r -= bias.angular_velocity.x*dt;
		// 	delta_p -= bias.angular_velocity.y*dt;
		// 	delta_y -= bias.angular_velocity.z*dt;
		// }
		tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
		q_pose = q_pose*q_relative_rotation;
		q_pose.normalize();
		quaternionTFToMsg(q_pose, odom.pose.pose.orientation);
	}
	Publication();

	imu_last = *msg;
	first_callback = false;
}

void ImuOdometry::Publication(void)
{
	/*publish*/
	pub_odom.publish(odom);
	/*tf broadcast*/
    // geometry_msgs::TransformStamped transform;
	// transform.header.stamp = ros::Time::now();
	// transform.header.frame_id = "/odom";
	// transform.child_frame_id = "/gyrodometry";
	// transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	// transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	// transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	// transform.transform.rotation = odom3d_now.pose.pose.orientation;
	// tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_odometry");

	ImuOdometry imu_odometry;

	ros::spin();
}
