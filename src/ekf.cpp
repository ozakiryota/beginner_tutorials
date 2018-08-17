/*
 * ekf.cpp
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

Eigen::MatrixXd X(6, 1);
Eigen::MatrixXd P(6, 6);

void input_pose(geometry_msgs::Pose& pose)
{
	tf::Quaternion q_ = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	quaternionTFToMsg(q_, pose.orientation);
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
}

void callback_observation_usingwalls(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	std::cout << "CALLBACK OBSERVATION USINGWALLS" << std::endl;
	if(inipose_is_available){
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_vector (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::fromROSMsg(*msg, *g_vector);

		const float g = -9.80665;
		double gx = g_vector->points[0].normal_x*g; 
		double gy = g_vector->points[0].normal_y*g; 
		double gz = g_vector->points[0].normal_z*g; 

		Eigen::MatrixXd Z(2, 1);
		Z <<	atan2(gy, gz),
		  		atan2(-gx, sqrt(gy*gy + gz*gz));
		// Z <<	atan2(g_vector->points[0].normal_y*g, g_vector->points[0].normal_z*g),
		// 		atan2(-g_vector->points[0].normal_x*g, sqrt(g_vector->points[0].normal_y*g*g_vector->points[0].normal_y*g + g_vector->points[0].normal_z*g*g_vector->points[0].normal_z*g));

		Eigen::MatrixXd H(2, 6);
		H <<	1,	0,	0,	0,	0,	0,
				0,	1,	0,	0,	0,	0;

		Eigen::MatrixXd jH(2, 6);
		jH <<	1,	0,	0,	0,	0,	0,
				0,	1,	0,	0,	0,	0;

		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);

		Eigen::MatrixXd R(2, 2);
		const double sigma = 1.0e+2;
		R = sigma*Eigen::MatrixXd::Identity(2, 2);

		Eigen::MatrixXd Y(2, 1);
		Eigen::MatrixXd S(2, 2);
		Eigen::MatrixXd K(12, 2);

		Y = Z - H*X;
		S = jH*P*jH.transpose() + R;
		K = P*H.transpose()*S.inverse();
		X = X + K*Y;
		P = (I - K*H)*P;
	}
}

void callback_observation_slam(const geometry_msgs::QuaternionConstPtr& msg)
{
	double roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
	
	Eigen::MatrixXd Z(3, 1);
	Z <<	roll,
			pitch,
			yaw;
	Eigen::MatrixXd H(3, 3);
	H <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd jH(3, 3);
	jH <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd I(3, 3);
	I <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXd R(3, 3);
	
	Eigen::MatrixXd Y(3, 1);
	Eigen::MatrixXd S(3, 3);
	Eigen::MatrixXd K(3, 3);
	
	Y = Z - H*X;
	S = jH*P*jH.transpose();
	K = P*H.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*H)*P;
}

void prediction(double dt)
{
	// std::cout << "PREDICTION" << std::endl;
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);
	double wx = imu.angular_velocity.x;
	double wy = imu.angular_velocity.y;
	double wz = imu.angular_velocity.z;
	
	Eigen::MatrixXd F(6, 1);
	F <<	roll + (wx + sin(roll)*tan(pitch)*wy + cos(roll)*tan(pitch)*wz)*dt,
	  		pitch + (cos(roll)*wy - sin(roll)*wz)*dt,
			yaw + (sin(roll)/cos(pitch)*wy + cos(roll)/cos(pitch)*wz)*dt,
			wx,
			wy,
			wz;

	double df0dx0 = 1.0 + (cos(roll)*tan(pitch)*wy - sin(roll)*tan(pitch)*wz)*dt;
	double df0dx1 = (sin(roll)/cos(pitch)/cos(pitch)*wy + cos(roll)/cos(pitch)/cos(pitch)*wz)*dt;
	double df0dx2 = 0.0;
	double df0dx3 = dt;
	double df0dx4 = sin(roll)*tan(pitch)*dt;
	double df0dx5 = cos(roll)*tan(pitch)*dt;
	double df1dx0 = (-sin(roll)*wy - cos(roll)*wz)*dt;
	double df1dx1 = 1.0;
	double df1dx2 = 0.0;
	double df1dx3 = 0.0;
	double df1dx4 = cos(roll)*dt;
	double df1dx5 = -sin(roll)*dt;
	double df2dx0 = (cos(roll)/cos(pitch)*wy - sin(roll)/cos(pitch)*wz)*dt;
	double df2dx1 = (-sin(roll)/sin(pitch)*wy - cos(roll)/sin(pitch)*wz)*dt;
	double df2dx2 = 1.0;
	double df2dx3 = 0.0;
	double df2dx4 = sin(roll)/cos(pitch)*dt;
	double df2dx5 = cos(roll)/cos(pitch)*dt;

	Eigen::MatrixXd jF(6, 6);
	jF <<	df0dx0,	df0dx1,	df0dx2,	df0dx3,	df0dx4,	df0dx5,
			df1dx0,	df1dx1,	df1dx2,	df1dx3,	df1dx4,	df1dx5,
			df2dx0,	df2dx1,	df2dx2,	df2dx3,	df2dx4,	df2dx5,
			0.0,	0.0,	0.0,	1.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	1.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	1.0;
		
	Eigen::MatrixXd Q(6, 6);
	const double sigma = 1.0e-2;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
	Q = sigma*I;

	X = F;
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
	ros::Subscriber sub_obs = nh.subscribe("/g_usingwalls", 10, callback_observation_usingwalls);
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>("/pose_estimation_", 1);

	X = Eigen::MatrixXd::Constant(6, 1, 0.0);
	P = 100.0*Eigen::MatrixXd::Identity(6, 6);

	while(ros::ok()){
		ros::spinOnce();
		input_pose(pose);
		pub_pose.publish(pose);
	}
}
