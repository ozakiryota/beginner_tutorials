/*
 *	imu_alignment.cpp
*/

#include <ros/ros.h>
#include <Eigen/Core>
// #include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include "beginner_tutorials/imudata.h"
#include <random>

sensor_msgs::Imu imu;
geometry_msgs::Quaternion initial_pose;
std::vector<beginner_tutorials::imudata> record;
beginner_tutorials::imudata ave;
ros::Time current_time;
ros::Time last_time;
bool imu_is_moving = false;
Eigen::MatrixXf X(3, 1);
Eigen::MatrixXf P(3, 3);

void input_initialpose(void)
{
	tf::Quaternion q = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	quaternionTFToMsg(q, initial_pose);
}

void observation(void)
{
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);
	Eigen::MatrixXf Rot(3, 3);
	Rot <<	cos(roll)*cos(pitch),	cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),	cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
			sin(roll)*cos(pitch),	sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),	sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
			-sin(pitch),	cos(pitch)*sin(yaw),	cos(pitch)*cos(yaw);
	
	Eigen::MatrixXf G(3, 1);
	G <<	0.0,
			0.0,
			9.80665;
	
	Eigen::MatrixXf H(3, 12);
	H <<	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0;
	
	Eigen::MatrixXf jH(3, 12);
	jH <<	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0;
	
	std::random_device rnd;
	std::mt19937 engine(rnd());
	const double sigma = 0.1;
	std::normal_distribution<double> dist(0.0, sigma);
	std::cout << "dist(sigma) = " << dist(sigma) << std::endl;
	Eigen::MatrixXf R(3, 3);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);
	R = sigma*I;
	Eigen::MatrixXf Z(3, 1);
	
	Z = Rot*G;
	Y = Z - H*X;
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;
}

void prediction(void)
{
	// double ax = imu.linear_acceleration.x - X(6, 0);
	// double ay = imu.linear_acceleration.y - X(7, 0);
	// double az = imu.linear_acceleration.z - X(8, 0);
	// double wx = imu.angular_velocity.x - X(9, 0);
	// double wy = imu.angular_velocity.y - X(10, 0);
	double ax = ave.ax - X(6, 0);
	double ay = ave.ay - X(7, 0);
	double az = ave.az - X(8, 0);
	double wx = ave.wx - X(9, 0);
	double wy = ave.wy - X(10, 0);
	
	Eigen::MatrixXf F(9, 1);
	F <<	atan2(ay, az),
			atan2(-ax, sqrt(ay*ay + az*az)),
			-atan2(wy, wx),
			imu.linear_acceleration.x,
			imu.linear_acceleration.y,
			imu.linear_acceleration.z,
			X(6, 0),
			X(7, 0),
			X(8, 0),
			X(9, 0),
			X(10, 0),
			X(11, 0);
	
	double dpdax = 1.0/(1.0 + (-ax/sqrt(ay*ay + az*az)*(-ax/sqrt(ay*ay + az*az))))*(-1.0/sqrt(ay*ay + az*az));
	double drday = 1.0/(1.0 + (ay/az)*(ay/az))*(1.0/az));
	double dpday = 1.0/(1.0 + (-ax/sqrt(ay*ay + az*az)*(-ax/sqrt(ay*ay + az*az))))*(ax/(2.0*(ay*ay + az*az)*sqrt(ay*ay + az*az)))*(2*ay);
	double drdaz = 1.0/(1.0 + (ay/az)*(ay/az))*(-ay/(az*az)));
	double dpdaz = 1.0/(1.0 + (-ax/sqrt(ay*ay + az*az)*(-ax/sqrt(ay*ay + az*az))))*(ax/(2.0*(ay*ay + az*az)*sqrt(ay*ay + az*az)))*(2*az);
	double dydwx = -1.0/(1.0 + (wy/wx)*(wy/wx))*(-wy/(wx*wx)));
	double dydwy = -1.0/(1.0 + (wy/wx)*(wy/wx))*(1.0/wx));

	Eigen::MatrixXf jF(12, 12);
	jF <<	1.0,	0.0,	0.0,	0.0,	drday,	drdaz,	0.0,	-drday,	-drdaz,	0.0,	0.0,	0.0,
			0.0,	1.0,	0.0,	dpdax,	dpday,	dpdaz,	-dpdax,	-dpday,	-dpdaz,	0.0,	0.0,	0.0,
			0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	dydwx,	dydwy,	0.0,
			0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0;
	
	std::random_device rnd;
	std::mt19937 engine(rnd());
	const double sigma = 1.0;
	std::normal_distribution<double> dist(0.0, sigma);
	std::cout << "dist(sigma) = " << dist(sigma) << std::endl;
	Eigen::MatrixXf Q(12, 12);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12,12);
	Q = sigma*I;
	std::cout << "Q = " << std::endl << Q << std::endl;
	
	X = F;
	P = jF*P*jF.transpose() + Q;
}

bool judge_moving(void)
{
	const float threshold_w = 1.0;
	const float threshold_a = 1.0;
	if(fabs(record[record.size()-1].wx - ave.wx)>threshold_w)	return true;
	if(fabs(record[record.size()-1].wy - ave.wy)>threshold_w)	return true;
	if(fabs(record[record.size()-1].wz - ave.wz)>threshold_w)	return true;
	if(fabs(record[record.size()-1].ax - ave.ax)>threshold_a)	return true;
	if(fabs(record[record.size()-1].ay - ave.ay)>threshold_a)	return true;
	if(fabs(record[record.size()-1].az - ave.az)>threshold_a)	return true;
	return false;
}

void calculate_average(void)
{
	ave.wx = 0.0;
	ave.wy = 0.0;
	ave.wz = 0.0;
	ave.ax = 0.0;
	ave.ay = 0.0;
	ave.az = 0.0;
	
	for(size_t i=0;i<record.size();i++){
		ave.wx += record[i].wx/(float)record.size();
		ave.wy += record[i].wy/(float)record.size();
		ave.wz += record[i].wz/(float)record.size();
		ave.ax += record[i].ax/(float)record.size();
		ave.ay += record[i].ay/(float)record.size();
		ave.az += record[i].az/(float)record.size();
	}
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "imu_callback" << std::endl;
	imu = *msg;
	
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;
	
	beginner_tutorials::imudata tmp;
	tmp.wx = imu.angular_velocity.x;
	tmp.wy = imu.angular_velocity.y;
	tmp.wz = imu.angular_velocity.z;
	tmp.ax = imu.linear_acceleration.x;
	tmp.ay = imu.linear_acceleration.y;
	tmp.az = imu.linear_acceleration.z;
	record.push_back(tmp);
	
	calculate_average();
	const size_t record_size = 100;
	if(record.size()>record_size){
		record.erase(record.begin());
		// calculate_average();
		imu_is_moving = judge_moving();
	}
	if(record.size()>record_size&&!imu_is_moving){
		prediction();
		observation();
	}
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_alignment");
	ros::NodeHandle nh;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Publisher pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/initial_pose",1);

	X = MatrixXf::Constant(12,1,0.0);
	std::cout << "X = " << std::endl << X << std::endl;
	
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12,12);
	P = 1000.0*I;
	std::cout << "P = " << std::endl << P << std::endl;

	while(ros::ok()&&!imu_is_moving){
		ros::spinOnce();
	}
	
	input_initialpose();
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		pub_inipose.publish(initial_pose);
		loop_rate.sleep();
	}
}