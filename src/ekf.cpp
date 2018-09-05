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
/*getParam*/
std::string PUB_POSE_NAME;
bool USE_SLAM;
bool USE_WALLS;

void input_pose(geometry_msgs::Pose& pose, Eigen::MatrixXd X)
{
	tf::Quaternion q_ = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	quaternionTFToMsg(q_, pose.orientation);
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;

	tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	pose.position.x = transform.getOrigin().x();
	pose.position.y = transform.getOrigin().y();
	pose.position.z = transform.getOrigin().z();
}

int count_usingwalls = 0;
void callback_observation_usingwalls(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	const int num_obs = 3;
	if(inipose_is_available&&USE_WALLS){
		std::cout << count_usingwalls << ": ";
		count_usingwalls++;
		std::cout << "CALLBACK OBSERVATION USINGWALLS" << std::endl;
		
		pcl::PointCloud<pcl::PointNormal>::Ptr g_vector (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(*msg, *g_vector);

		const double g = -9.80665;
		double gx = g_vector->points[0].normal_x*g; 
		double gy = g_vector->points[0].normal_y*g; 
		double gz = g_vector->points[0].normal_z*g; 
		// std::cout << "G = " << gx << " " << gy << " " << gz << std::endl;

		Eigen::MatrixXd Z(num_obs, 1);
		Z <<	atan2(gy, gz),
		  		atan2(-gx, sqrt(gy*gy + gz*gz)),
				X(2, 0);

		Eigen::MatrixXd H(num_obs, num_state);
		H <<	1,	0,	0,
				0,	1,	0,
				0, 	0,	1;

		Eigen::MatrixXd jH(num_obs, num_state);
		jH <<	1,	0,	0,
				0,	1,	0,
				0,	0,	1;

		Eigen::MatrixXd R(num_obs, num_obs);
		const double sigma = 1.0e-20;
		R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);

		Eigen::MatrixXd Y(num_obs, 1);
		Eigen::MatrixXd S(num_obs, num_obs);
		Eigen::MatrixXd K(num_state, num_obs);
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

		// std::cout << "X_pre = " << std::endl << X << std::endl;
		// std::cout << "P_pre = " << std::endl << P << std::endl;
		
		Y = Z - H*X;
		S = jH*P*jH.transpose() + R;
		K = P*jH.transpose()*S.inverse();
		X = X + K*Y;
		P = (I - K*jH)*P;

		// std::cout << "K = " << std::endl << K << std::endl;
		// std::cout << "K*Y = " << std::endl << K*Y << std::endl;
		// std::cout << "I - K*jH = " << std::endl << I - K*jH << std::endl;
		// std::cout << "X_obs = " << std::endl << X << std::endl;
		// std::cout << "P_obs_walls = " << std::endl << P << std::endl;
	}
}

tf::Quaternion q_last;
Eigen::MatrixXd X_last(num_state, 1);
int count_slam = 0;
void callback_observation_slam(const geometry_msgs::PoseStampedConstPtr& msg)
{
	const int num_obs = 3;

	tf::Quaternion q_now(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
	// if(!inipose_is_available){
	// 	q_last = q_now;
	// 	X_last = X;
	// }
	if(inipose_is_available)	count_slam++;
	// if(!inipose_is_available)	pose_slam_last = msg->pose.orientation;
	// if(USE_SLAM && inipose_is_available && count_slam>100){
	if(USE_SLAM && inipose_is_available){
		if(count_slam==101)	std::cout << "CALLBACK OBSERVATION SLAM" << std::endl;
		// std::cout << "CALLBACK OBSERVATION SLAM" << std::endl;
		// tf::Quaternion q_now(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
		// tf::Quaternion q_last(pose_slam_last.z, -pose_slam_last.x, -pose_slam_last.y, pose_slam_last.w);
		// tf::Quaternion rot_q = q_slam*q_slam.inverse();
		// pose_slam_last = msg->pose.orientation;

		// tf::Quaternion x_now = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
		tf::Quaternion x_last = tf::createQuaternionFromRPY(X_last(0, 0), X_last(1, 0), X_last(2, 0));

		double roll, pitch, yaw;
		// double roll_, pitch_, yaw_;
		tf::Matrix3x3((q_now*q_last.inverse())*x_last).getRPY(roll, pitch, yaw);	//planA
		// tf::Matrix3x3(q_now).getRPY(roll, pitch, yaw);			//planB
		// tf::Matrix3x3(q_last).getRPY(roll_, pitch_, yaw_);

		Eigen::MatrixXd Z(num_obs, 1);
		Z <<	roll,
				pitch,
				yaw;
		// Z <<	X_last(0, 0) + (roll - roll_),
		// 		X_last(1, 0) + (pitch - pitch_),
		// 		X_last(2, 0) + (yaw - yaw_);
		q_last = q_now;
		X_last = X;
		
		Eigen::MatrixXd H(num_obs, num_state);
		H <<	1,	0,	0,
				0,	1,	0,
				0,	0,	1;
		Eigen::MatrixXd jH(num_obs, num_state);
		jH <<	1,	0,	0,
				0,	1,	0,
				0,	0,	1;
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);
		const double sigma = 1.0e-4;
		Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);

		Eigen::MatrixXd Y(3, 1);
		Eigen::MatrixXd S(3, 3);
		Eigen::MatrixXd K(3, 3);

		Y = Z - H*X;
		S = jH*P*jH.transpose() + R;
		K = P*H.transpose()*S.inverse();
		X = X + K*Y;
		P = (I - K*H)*P;

		// std::cout << "K*Y = " << std::endl << K*Y << std::endl;
		// std::cout << "P_obs_slam = " << std::endl << P << std::endl;
	}
	else{
		q_last = q_now;
		X_last = X;
	}
	/* testing */
	// tf::Quaternion q_now_(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
	// if(!inipose_is_available)	q_last = q_now_;
	// else{
	// 	double roll, pitch, yaw;
	// 	double roll_, pitch_, yaw_;
	// 	tf::Quaternion x = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	// 	// tf::Matrix3x3((q_now_*q_last.inverse())*x).getRPY(roll, pitch, yaw);
	// 	tf::Matrix3x3(q_now_).getRPY(roll, pitch, yaw);
	// 	tf::Matrix3x3(q_last).getRPY(roll_, pitch_, yaw_);
	// 	X(0, 0) += roll - roll_;
	// 	X(1, 0) += pitch - pitch_;
	// 	X(2, 0) += yaw - yaw_;
	// 	q_last = q_now_;
	// }
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
	
	Eigen::MatrixXd F(num_state, 1);
	F <<	roll + (wx + sin(roll)*tan(pitch)*wy + cos(roll)*tan(pitch)*wz)*dt,
	  		pitch + (cos(roll)*wy - sin(roll)*wz)*dt,
			yaw + (sin(roll)/cos(pitch)*wy + cos(roll)/cos(pitch)*wz)*dt;

	double df0dx0 = 1.0 + (cos(roll)*tan(pitch)*wy - sin(roll)*tan(pitch)*wz)*dt;
	double df0dx1 = (sin(roll)/cos(pitch)/cos(pitch)*wy + cos(roll)/cos(pitch)/cos(pitch)*wz)*dt;
	double df0dx2 = 0.0;
	double df1dx0 = (-sin(roll)*wy - cos(roll)*wz)*dt;
	double df1dx1 = 1.0;
	double df1dx2 = 0.0;
	double df2dx0 = (cos(roll)/cos(pitch)*wy - sin(roll)/cos(pitch)*wz)*dt;
	double df2dx1 = (-sin(roll)/sin(pitch)*wy - cos(roll)/sin(pitch)*wz)*dt;
	double df2dx2 = 1.0;

	Eigen::MatrixXd jF(num_state, num_state);
	jF <<	df0dx0,	df0dx1,	df0dx2,
			df1dx0,	df1dx1,	df1dx2,
			df2dx0,	df2dx1,	df2dx2;	
	Eigen::MatrixXd Q(num_state, num_state);
	// const double sigma = 1.0e+0;
	const double sigma = 0.0;
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);
	Q = sigma*I;

	X = F;
	P = jF*P*jF.transpose() + Q;

	if(X(2, 0)>M_PI)	X(2, 0) -= 2*M_PI;
	if(X(2, 0)<-M_PI)	X(2, 0) += 2*M_PI;

	// std::cout << "P_pre = " << std::endl << P << std::endl;
	// std::cout << "F = " << std::endl << F << std::endl;
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
	
	ros::NodeHandle local_nh("~");
	local_nh.getParam("PUB_POSE_NAME", PUB_POSE_NAME);
	local_nh.getParam("USE_SLAM", USE_SLAM);
	local_nh.getParam("USE_WALLS", USE_WALLS);

	geometry_msgs::Pose pose;
	
	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Subscriber sub_obs1 = nh.subscribe("/g_usingwalls", 10, callback_observation_usingwalls);
	ros::Subscriber sub_obs2 = nh.subscribe("/lsd_slam/pose", 10, callback_observation_slam);
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>(PUB_POSE_NAME, 1);

	X = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	P = 100.0*Eigen::MatrixXd::Identity(num_state, num_state);

	while(ros::ok()){
		ros::spinOnce();
		input_pose(pose, X);
		pub_pose.publish(pose);
	}
}
