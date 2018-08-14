/*
 *	imu_alignment.cpp
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
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
Eigen::MatrixXd X(12, 1);
Eigen::MatrixXd P(12, 12);
FILE* fp;

void input_initialpose(void)
{
	std::cout << "INPUT INITIALPOSE" << std::endl;
	tf::Quaternion q = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	quaternionTFToMsg(q, initial_pose);
}

Eigen::MatrixXd rotation(Eigen::MatrixXd X, double roll, double pitch, double yaw)
{
	geometry_msgs::Quaternion q;
	tf::Quaternion q_ = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	quaternionTFToMsg(q_, q);
	Eigen::MatrixXd Rot(3, 3);
	Rot <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,	2*(q.x*q.y - q.w*q.z),	2*(q.x*q.z + q.w*q.y),
			2*(q.x*q.y + q.w*q.z),  q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,  2*(q.y*q.z - q.w*q.x),
			2*(q.x*q.z - q.w*q.y),  2*(q.y*q.z + q.w*q.x),  q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	return Rot*X;
}

// void observation0(void)
// {
// 	double roll = X(0, 0);
// 	double pitch = X(1, 0);
// 	double yaw = X(2, 0);
// 	// Eigen::MatrixXd Rot(3, 3);
// 	// Rot <<	cos(roll)*cos(pitch),	cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),	cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
// 	// 		sin(roll)*cos(pitch),	sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),	sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
// 	// 		-sin(pitch),	cos(pitch)*sin(yaw),	cos(pitch)*cos(yaw);
//
// 	Eigen::MatrixXd Rot(3, 3);
// 	Rot <<	cos(pitch)*cos(yaw),
// 				cos(pitch)*sin(yaw),
// 					-sin(pitch),
// 			sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
// 				sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
// 					sin(roll)*cos(pitch),
// 			cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
// 				cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
// 					cos(roll)*cos(pitch);
// 	// Eigen::MatrixXd Rot(3, 3);
// 	// Rot <<	0.0,	0.0,	sin(X(1, 0)),
// 	// 		0.0,	0.0,	-sin(X(0, 0))*cos(X(1, 0)),
// 	// 		0.0,	0.0,	-cos(X(0, 0))*cos(X(1, 0));
// 	
// 	Eigen::MatrixXd G(3, 1);
// 	G <<	0.0,
// 			0.0,
// 			9.80665;
//
// 	Eigen::MatrixXd H(3, 12);
// 	H <<	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
// 			0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,
// 			0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0;
// 	
// 	const double g = 9.80665;
// 	double daydr = -g*cos(roll)*cos(pitch);
// 	double dazdr = g*sin(roll)*cos(pitch);
// 	double daxdp = g*cos(pitch);
// 	double daydp = g*sin(roll)*sin(pitch);
// 	double dazdp = g*cos(roll)*sin(pitch);
// 	Eigen::MatrixXd jH(3, 12);
// 	jH <<	0.0,	daxdp,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
// 			daydr,	daydp,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,
// 			dazdr,	dazdp,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0;
//
//
// 	// Eigen::MatrixXd H(3, 12);
// 	// H <<	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
// 	// 		0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,
// 	// 		0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0;
// 	// Eigen::MatrixXd jH(3, 12);
// 	// jH <<	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,	0.0,
// 	// 		0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0,	0.0,
// 	// 		0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	-1.0,	0.0,	0.0,	0.0;
// 	
// 	std::random_device rnd;
// 	std::mt19937 engine(rnd());
// 	const double sigma = 0.01;
// 	std::normal_distribution<double> dist(0.0, sigma);
// 	// std::cout << "obs_dist(engine) = " << dist(engine) << std::endl;
// 	Eigen::MatrixXd R(3, 3);
// 	Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
// 	R = sigma*I3;
// 	Eigen::MatrixXd Z(3, 1);
// 	Eigen::MatrixXd Y(3, 1);
// 	Eigen::MatrixXd S(3, 3);
// 	Eigen::MatrixXd K(12, 3);
// 	Eigen::MatrixXd I12 = Eigen::MatrixXd::Identity(12, 12);
// 	
// 	// Z = rotation(G, X(0, 0), X(1, 0), X(2, 0));
// 	// std::cout << "Z = " << std::endl << Z << std::endl;
// 	Z = Rot*G;
// 	// std::cout << "inverse = " << std::endl << Rot.inverse()*Z << std::endl;
// 	std::cout << "inverse = " << std::endl << Rot.inverse()*Z << std::endl;
// 	Y = Z - H*X;
// 	S = jH*P*jH.transpose() + R;
// 	K = P*jH.transpose()*S.inverse();
// 	X = X + K*Y;
// 	P = (I12 - K*jH)*P;
//
// 	std::cout << "Z = " << std::endl << Z << std::endl;
// 	std::cout << "H*X = " << std::endl << H*X << std::endl;
// 	// std::cout << "Y = " << std::endl << Y << std::endl;
// 	// std::cout << "S = " << std::endl << S << std::endl;
// 	// std::cout << "S.inverse() = " << std::endl << S.inverse() << std::endl;
// 	// std::cout << "K = " << std::endl << K << std::endl;
// 	std::cout << "X_obs = " << std::endl << X << std::endl;
// 	std::cout << "P_obs = " << std::endl << P << std::endl;
// }

void observation(void)
{
	double ax = X(3, 0) - X(6, 0);
	double ay = X(4, 0) - X(7, 0);
	double az = X(5, 0) - X(8, 0);
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);
	
	Eigen::MatrixXd Acc(3, 1);
	Acc <<	ax,
			ay,
			az;
	Eigen::MatrixXd Rot(3, 3);
	Rot <<	cos(pitch)*cos(yaw),
				sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
					cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
			cos(pitch)*sin(yaw),
				sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
					cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
			-sin(pitch),
				sin(roll)*cos(pitch),
					cos(roll)*cos(pitch);
	
	const double g = 9.80665;
	Eigen::MatrixXd Z(3, 1);
	Z <<	0.0,
			0.0,
			g;

	double dh0dx0 = (cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw))*ay
					+(-sin(roll)*sin(pitch)*cos(yaw) + cos(roll)*sin(yaw))*az;
	double dh0dx1 = -sin(pitch)*cos(yaw)*ax
					+sin(roll)*cos(pitch)*cos(yaw)*ay
					+cos(roll)*cos(pitch)*cos(yaw)*az;
	double dh0dx2 =	-cos(pitch)*sin(yaw)*ax
					+(-sin(roll)*sin(pitch)*sin(yaw) - cos(roll)*cos(yaw))*ay
					+(-cos(roll)*sin(pitch)*sin(yaw) + sin(roll)*cos(yaw))*az;
	double dh1dx0 =	(cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw))*ay
					+(-sin(roll)*sin(pitch)*sin(yaw) - cos(roll)*cos(yaw))*az;
	double dh1dx1 = -sin(pitch)*sin(yaw)*ax
					+sin(roll)*cos(pitch)*sin(yaw)*ay
					+cos(roll)*cos(pitch)*sin(yaw)*az;
	double dh1dx2 =	cos(pitch)*cos(yaw)*ax
					+(sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw))*ay
					+(cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw))*az;
	double dh2dx0 = cos(roll)*cos(pitch)*ay
					-sin(roll)*cos(pitch)*az;
	double dh2dx1 = -cos(pitch)*ax
					-sin(roll)*sin(pitch)*ay
					-cos(roll)*sin(pitch)*az;
	double dh2dx2 = 0.0;
	double dh0dx3 = cos(pitch)*cos(yaw);
	double dh0dx4 = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
	double dh0dx5 = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
	double dh1dx3 = cos(pitch)*sin(yaw);
	double dh1dx4 = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
	double dh1dx5 = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
	double dh2dx3 = -sin(pitch);
	double dh2dx4 = sin(roll)*cos(pitch);
	double dh2dx5 = cos(roll)*cos(pitch);
	double dh0dx6 = -dh0dx3;	
	double dh0dx7 = -dh0dx4;	
	double dh0dx8 = -dh0dx5;
	double dh1dx6 = -dh1dx3;	
	double dh1dx7 = -dh1dx4;	
	double dh1dx8 = -dh1dx5;	
	double dh2dx6 = -dh2dx3;
	double dh2dx7 = -dh2dx4;	
	double dh2dx8 = -dh2dx5;

	Eigen::MatrixXd jH(3, 12);
	jH <<	dh0dx0,	dh0dx1,	dh0dx2,	dh0dx3,	dh0dx4,	dh0dx5,	dh0dx6,	dh0dx7,	dh0dx8,	0.0,	0.0,	0.0,
			dh1dx0,	dh1dx1,	dh1dx2,	dh1dx3,	dh1dx4,	dh1dx5,	dh1dx6,	dh1dx7,	dh1dx8,	0.0,	0.0,	0.0,
			dh2dx0,	dh2dx1,	dh2dx2,	dh2dx3,	dh2dx4,	dh2dx5,	dh2dx6,	dh2dx7,	dh2dx8,	0.0,	0.0,	0.0;
	
	std::random_device rnd;
	std::mt19937 engine(rnd());
	const double sigma = 0.001;
	std::normal_distribution<double> dist(0.0, sigma);
	std::cout << "obs_dist(engine) = " << dist(engine) << std::endl;
	Eigen::MatrixXd R(3, 3);
	Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3, 3);
	R = sigma*I3;
	Eigen::MatrixXd H(3, 1);
	Eigen::MatrixXd Y(3, 1);
	Eigen::MatrixXd S(3, 3);
	Eigen::MatrixXd K(12, 3);
	Eigen::MatrixXd I12 = Eigen::MatrixXd::Identity(12, 12);
	

	// Z = Rot*G;
	H = Rot*Acc;
	Y = Z - H;
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I12 - K*jH)*P;

	// std::cout << "Acc = " << std::endl << Acc << std::endl;
	// std::cout << "H = " << std::endl << H << std::endl;
	std::cout << "Y = " << std::endl << Y << std::endl;
	// std::cout << "S = " << std::endl << S << std::endl;
	// std::cout << "S.inverse() = " << std::endl << S.inverse() << std::endl;
	// std::cout << "K = " << std::endl << K << std::endl;
	// std::cout << "X_obs = " << std::endl << X << std::endl;
	// std::cout << "P_obs = " << std::endl << P << std::endl;
	
	fprintf(fp, "%f\t%f\t%f\n", Y(0, 0), Y(1, 0), Y(2, 0));
	// fprintf(fp, "%f\t%f\t%f\n", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
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

	double roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);

	Eigen::MatrixXd F(12, 1);
	F <<	atan2(ay, az),
			atan2(-ax, sqrt(ay*ay + az*az)),
			yaw,	// -atan2(wy, wx),
			imu.linear_acceleration.x,
			imu.linear_acceleration.y,
			imu.linear_acceleration.z,
			X(6, 0),
			X(7, 0),
			X(8, 0),
			X(9, 0),
			X(10, 0),
			X(11, 0);
	
	double drday = 1.0/(1.0 + (ay/az)*(ay/az))*(1.0/az);
	double drdaz = 1.0/(1.0 + (ay/az)*(ay/az))*(-ay/(az*az));
	double dpdax = 1.0/(1.0 + (-ax/sqrt(ay*ay + az*az)*(-ax/sqrt(ay*ay + az*az))))*(-1.0/sqrt(ay*ay + az*az));
	double dpday = 1.0/(1.0 + (-ax/sqrt(ay*ay + az*az)*(-ax/sqrt(ay*ay + az*az))))*(ax/(2.0*(ay*ay + az*az)*sqrt(ay*ay + az*az)))*(2.0*ay);
	double dpdaz = 1.0/(1.0 + (-ax/sqrt(ay*ay + az*az)*(-ax/sqrt(ay*ay + az*az))))*(ax/(2.0*(ay*ay + az*az)*sqrt(ay*ay + az*az)))*(2.0*az);
	double dydwx = -1.0/(1.0 + (wy/wx)*(wy/wx))*(-wy/(wx*wx));
	double dydwy = -1.0/(1.0 + (wy/wx)*(wy/wx))*(1.0/wx);

	Eigen::MatrixXd jF(12, 12);
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
	const double sigma = 0.001;
	std::normal_distribution<double> dist(0.0, sigma);
	// std::cout << "pre_dist(engine) = " << dist(engine) << std::endl;
	Eigen::MatrixXd Q(12, 12);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12, 12);
	Q = sigma*I;
	
	X = F;
	P = jF*P*jF.transpose() + Q;
	// std::cout << "X_pre = " << std::endl << X << std::endl;
	// std::cout << "P_pre = " << std::endl << P << std::endl;
}

bool judge_moving(void)
{
	const float threshold_w = 0.03;
	const float threshold_a = 0.03;
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
	if(record.size()==record_size&&!imu_is_moving){
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

	X = Eigen::MatrixXd::Constant(12, 1, 0.0);
	std::cout << "X = " << std::endl << X << std::endl;
	
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12, 12);
	P = 100.0*I;
	std::cout << "P = " << std::endl << P << std::endl;

	fp = fopen("/home/amsl/Desktop/imu_alignment.csv", "w");

	while(ros::ok()&&!imu_is_moving){
		ros::spinOnce();
	}
	
	fclose(fp);
	input_initialpose();
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		pub_inipose.publish(initial_pose);
		loop_rate.sleep();
	}
}
