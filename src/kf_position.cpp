/*
 * ekf.cpp
 *
 * X = |Φ Θ Ψ|^T
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>

sensor_msgs::Imu imu;
ros::Time current_time;
ros::Time last_time;
bool inipose_is_available = false;
const int num_state = 3;
Eigen::MatrixXd X(num_state, 1);
Eigen::MatrixXd P(num_state, num_state);
geometry_msgs::Quaternion pose_slam_last;

void input_pose(geometry_msgs::Pose& pose)
{
	tf::Quaternion q_ = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
	quaternionTFToMsg(q_, pose.orientation);
	pose.position.x = X(0, 0);
	pose.position.y = X(1, 0);
	pose.position.z = X(2, 0);

	// tf::TransformListener listener;
	// tf::StampedTransform transform;
	// try {
	// 	listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
	// 	listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
	// }
	// catch (tf::TransformException ex){
	// 	ROS_ERROR("%s",ex.what());
	// }
	// pose.position.x = transform.getOrigin().x();
	// pose.position.y = transform.getOrigin().y();
	// pose.position.z = transform.getOrigin().z();
}

void callback_slam(const geometry_msgs::PoseStampedConstPtr& msg)
{
	const int num_obs = 3;

	if(inipose_is_available){
		std::cout << "CALLBACK SLAM" << std::endl;
	
		Eigen::MatrixXd Z(num_obs, 1);
		Z <<	msg->pose.position.x,
				msg->pose.position.y,
				msg->pose.position.z;
		Eigen::MatrixXd H(num_obs, num_state);
		H <<	1,	0,	0,
				0,	1,	0,
				0,	0,	1;
		Eigen::MatrixXd jH(num_obs, num_state);
		jH <<	1,	0,	0,
				0,	1,	0,
				0,	0,	1;
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);
		const double sigma = 1.0e+0;
		Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);

		Eigen::MatrixXd Y(3, 1);
		Eigen::MatrixXd S(3, 3);
		Eigen::MatrixXd K(3, 3);
	
		Y = Z - H*X;
		S = jH*P*jH.transpose() + R;
		K = P*H.transpose()*S.inverse();
		X = X + K*Y;
		P = (I - K*H)*P;
	}
}

Eigen::MatrixXd V = Eigen::MatrixXd::Constant(3, 1, 0.0);
void prediction(double dt)
{
	// std::cout << "PREDICTION" << std::endl;
	Eigen::MatrixXd A(num_state, num_state);
	A <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd jF(num_state, num_state);
	jF <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd Acc(3, 1);
	Acc <<	-imu.linear_acceleration.x,
			-imu.linear_acceleration.y,
			-imu.linear_acceleration.z;
	Eigen::MatrixXd G(3, 1);
	G <<	0.0,
	  		0.0,
			-9.80665;
	V = V + (Acc - G)*dt;
	Eigen::MatrixXd Q(num_state, num_state);
	const double sigma = 1.0e-2;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);
	Q = sigma*I;

	X = A*X + V*dt;
	P = jF*P*jF.transpose() + Q;
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "imu_callback" << std::endl;
	imu = *msg;

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	if(inipose_is_available)	prediction(dt);
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		tf::Quaternion tmp_q(msg->x, msg->y, msg->z, msg->w);
		tf::Matrix3x3(tmp_q).getRPY(X(0, 0), X(1, 0), X(2, 0));
		inipose_is_available = true;
	}
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;

	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	geometry_msgs::Pose pose;

	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Subscriber sub_obs2 = nh.subscribe("/lsd_slam/pose", 10, callback_slam);
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>("/position_estimation_", 1);

	X = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	P = 100.0*Eigen::MatrixXd::Identity(num_state, num_state);

	while(ros::ok()){
		ros::spinOnce();
		input_pose(pose);
		pub_pose.publish(pose);
	}
}
