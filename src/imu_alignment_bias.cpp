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
#include <std_msgs/Float64.h>

geometry_msgs::Quaternion initial_pose;
// beginner_tutorials::imudata ave;
// beginner_tutorials::imudata bias;
bool imu_is_moving = false;
const int num_state = 3;
Eigen::MatrixXd X(num_state, 1);
Eigen::MatrixXd P(num_state, num_state);

std::vector<sensor_msgs::Imu> record;
sensor_msgs::Imu average;
ros::Time time_started;

void input_initialpose(void)
{
	std::cout << "INPUT INITIALPOSE" << std::endl;
	tf::Quaternion q = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
	// q = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
	quaternionTFToMsg(q, initial_pose);
	// inipose_is_available = true;
	// initial_pose = imu.orientation;
	std::cout << "initial pose = " << std::endl << X << std::endl;
}

Eigen::MatrixXd rotation(Eigen::MatrixXd X, double roll, double pitch, double yaw, bool global_to_local)
{
	tf::Quaternion q_ = tf::createQuaternionFromRPY(roll, pitch, yaw);
	// if(!local_to_global)	q_ = q_.inverse();
	geometry_msgs::Quaternion q;
	quaternionTFToMsg(q_, q);
	// if(!local_to_global)	q.w *= -1.0;
	Eigen::MatrixXd Rot(3, 3);
	Rot <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,	2*(q.x*q.y + q.w*q.z),	2*(q.x*q.z - q.w*q.y),
			2*(q.x*q.y - q.w*q.z),  q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,  2*(q.y*q.z + q.w*q.x),
			2*(q.x*q.z + q.w*q.y),  2*(q.y*q.z - q.w*q.x),  q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

	if(global_to_local){
		std::cout << "Rot*X = " << std::endl << Rot*X << std::endl;
		return Rot*X;
	}
	else{
		std::cout << "Rot.inverse()*X = " << std::endl << Rot.inverse()*X << std::endl;
		return Rot.inverse()*X;
	}
}
//
// void rotation_(Eigen::MatrixXd X, double roll, double pitch, double yaw, bool local_to_global)
// {
// 	tf::Quaternion q(X(0, 0), X(1, 0), X(2, 0), 1.0);
// 	tf::Quaternion rot = tf::createQuaternionFromRPY(roll, pitch, yaw);
// 	q = rot*q*rot.inverse();
//
// 	geometry_msgs::Quaternion q_;
// 	quaternionTFToMsg(q, q_);
// 	std::cout << "q_ = " << std::endl << q_ << std::endl;
// }

void observation_(void)
{
	const int num_obs = 3;	

	bool trans = false;

	double ax = average.linear_acceleration.x;
	double ay = average.linear_acceleration.y;
	double az = average.linear_acceleration.z;
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);
	
	Eigen::MatrixXd Rot(3, 3);
	Rot <<	cos(pitch)*cos(yaw),
				cos(pitch)*sin(yaw),
					-sin(pitch),
			sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
				sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
					sin(roll)*cos(pitch),
			cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
				cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
					cos(roll)*cos(pitch);
	
	const double g = 9.80665;
	Eigen::MatrixXd G(3, 1);
	G <<	0.0,
	  		0.0,
			g;
	
	Eigen::MatrixXd Z(num_obs, 1);
	Z <<	ax,
			ay,
			az;

	double dhdx[num_obs][num_state];
	dhdx[0][0] = 0.0;
	dhdx[0][1] = -cos(pitch)*g;
	dhdx[0][2] = 0.0;
	dhdx[1][0] = cos(roll)*cos(pitch)*g;
	dhdx[1][1] = -sin(roll)*sin(pitch)*g;
	dhdx[1][2] = 0.0;
	dhdx[2][0] = -sin(roll)*cos(pitch)*g;
	dhdx[2][1] = -cos(roll)*sin(pitch)*g;
	dhdx[2][2] = 0.0;

	if(trans){
		dhdx[0][0] = (-sin(roll)*sin(pitch)*cos(yaw) + cos(roll)*sin(yaw))*g;
		dhdx[0][1] = (cos(roll)*cos(pitch)*cos(yaw) + sin(roll)*sin(yaw))*g;
		dhdx[0][2] = (-cos(roll)*sin(pitch)*sin(yaw) + sin(roll)*cos(yaw))*g;
		dhdx[1][0] = (-sin(roll)*sin(pitch)*sin(yaw) - cos(roll)*cos(yaw))*g;
		dhdx[1][1] = (cos(roll)*cos(pitch)*sin(yaw))*g;
		dhdx[1][2] = (cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw))*g;
		dhdx[2][0] = -sin(roll)*cos(pitch)*g;
		dhdx[2][1] = -cos(roll)*sin(pitch)*g;
		dhdx[2][2] = 0.0;
	}

	Eigen::MatrixXd jH(num_obs, num_state);
	for(int i=0;i<num_obs;i++){
		for(int j=0;j<num_state;j++)	jH(i, j) = dhdx[i][j];
	}

	std::random_device rnd;
	std::mt19937 engine(rnd());
	const double sigma = 1.0e-4;
	std::normal_distribution<double> dist(0.0, sigma);
	// std::cout << "obs_dist(engine) = " << dist(engine) << std::endl;
	Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);
	Eigen::MatrixXd H(num_obs, 1);
	Eigen::MatrixXd Y(num_obs, 1);
	Eigen::MatrixXd S(num_obs, num_obs);
	Eigen::MatrixXd K(num_state, num_obs);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

	rotation(G, X(0, 0), X(1, 0), X(2, 0), true);
	H = Rot*G;
	if(trans)	H = Rot.transpose()*G;
	Y = Z - H;
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;

	// std::cout << "H = " << std::endl << H << std::endl;
	// std::cout << "jH = " << std::endl << jH << std::endl;
	// std::cout << "Y = " << std::endl << Y << std::endl;
	// std::cout << "S = " << std::endl << S << std::endl;
	// std::cout << "S.inverse() = " << std::endl << S.inverse() << std::endl;
	// std::cout << "K = " << std::endl << K << std::endl;
	// std::cout << "K*Y = " << std::endl << K*Y << std::endl;
	// std::cout << "Rot = " << std::endl << Rot << std::endl;
	// std::cout << "X_obs = " << std::endl << X << std::endl;
	// std::cout << "P_obs = " << std::endl << P << std::endl;
	
	// fprintf(fp, "%f\t%f\t%f\n", Y(0, 0), Y(1, 0), Y(2, 0));
	// fprintf(fp, "%f\t%f\t%f\n", imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
}

void observation(void)
{
	const int num_obs = 3;	

	double ax = average.linear_acceleration.x;
	double ay = average.linear_acceleration.y;
	double az = average.linear_acceleration.z;
	double roll = X(0, 0);
	double pitch = X(1, 0);
	double yaw = X(2, 0);
	
	Eigen::MatrixXd Z(num_obs, 1);
	Z <<	atan2(ay, az),
			atan2(-ax, sqrt(ay*ay + az*az)),
			yaw;

	Eigen::MatrixXd H(num_obs, num_state);
	H <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;

	Eigen::MatrixXd jH(num_obs, num_state);
	jH <<	1,	0,	0,
	  		0,	1,	0,
			0,	0,	1;

	std::random_device rnd;
	std::mt19937 engine(rnd());
	const double sigma = 1.0e-1;
	std::normal_distribution<double> dist(0.0, sigma);
	// std::cout << "obs_dist(engine) = " << dist(engine) << std::endl;
	Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);
	Eigen::MatrixXd Y(num_obs, 1);
	Eigen::MatrixXd S(num_obs, num_obs);
	Eigen::MatrixXd K(num_state, num_obs);
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

	Y = Z - H*X;
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;

	// std::cout << "K*Y = " << std::endl << K*Y << std::endl;
}
	
void prediction(void)
{	
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_state, num_state);
	Eigen::MatrixXd F(num_state, num_state);
	Eigen::MatrixXd jF = Eigen::MatrixXd::Identity(num_state, num_state);
	
	std::random_device rnd;
	std::mt19937 engine(rnd());
	const double sigma = 1.0e-1;
	std::normal_distribution<double> dist(0.0, sigma);
	// std::cout << "pre_dist(engine) = " << dist(engine) << std::endl;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(num_state, num_state);

	F = A*X;
	X = F;
	P = jF*P*jF.transpose() + Q;
	// std::cout << "X_pre = " << std::endl << X << std::endl;
	// std::cout << "P_pre = " << std::endl << P << std::endl;
}

bool judge_moving(void)
{
	const float threshold_w = 0.03;
	const float threshold_a = 0.07;
	if(fabs(record[record.size()-1].angular_velocity.x - average.angular_velocity.x)>threshold_w){
		std::cout << "Moved-wx" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].angular_velocity.y - average.angular_velocity.y)>threshold_w){
		std::cout << "Moved-wy" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].angular_velocity.z - average.angular_velocity.z)>threshold_w){
		std::cout << "Moved-wz" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.x - average.linear_acceleration.x)>threshold_a){
		std::cout << "Moved-ax" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.y - average.linear_acceleration.y)>threshold_a){
		std::cout << "Moved-ay" << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.z - average.linear_acceleration.z)>threshold_a){
		std::cout << "Moved-az" << std::endl;
		return true;
	}
	return false;
}

void calculate_average(void)
{
	average.angular_velocity.x = 0.0;
	average.angular_velocity.y = 0.0;
	average.angular_velocity.z = 0.0;
	average.linear_acceleration.x = 0.0;
	average.linear_acceleration.y = 0.0;
	average.linear_acceleration.z = 0.0;
	
	for(size_t i=0;i<record.size();i++){
		// average.angular_velocity.x += record[i].angular_velocity.x/(double)record.size();
		// average.angular_velocity.y += record[i].angular_velocity.y/(double)record.size();
		// average.angular_velocity.z += record[i].angular_velocity.z/(double)record.size();
		// average.linear_acceleration.x += record[i].linear_acceleration.x/(double)record.size();
		// average.linear_acceleration.y += record[i].linear_acceleration.y/(double)record.size();
		// average.linear_acceleration.z += record[i].linear_acceleration.z/(double)record.size();
		average.angular_velocity.x += record[i].angular_velocity.x;
		average.angular_velocity.y += record[i].angular_velocity.y;
		average.angular_velocity.z += record[i].angular_velocity.z;
		average.linear_acceleration.x += record[i].linear_acceleration.x;
		average.linear_acceleration.y += record[i].linear_acceleration.y;
		average.linear_acceleration.z += record[i].linear_acceleration.z;
	}
	average.angular_velocity.x /=(double)record.size();
	average.angular_velocity.y /= (double)record.size();
	average.angular_velocity.z /= (double)record.size();
	average.linear_acceleration.x /= (double)record.size();
	average.linear_acceleration.y /= (double)record.size();
	average.linear_acceleration.z /= (double)record.size();
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "imu_callback" << std::endl;
	if(record.size()==0)	time_started = ros::Time::now();

	record.push_back(*msg);
	calculate_average();
	imu_is_moving = judge_moving();
	
	if(!imu_is_moving){
		prediction();
		observation();
	}

	// const int datasize = 6000;
	// if(record.size()>datasize){
	// 	std::cout << "record.size()>" << datasize << std::endl;
	// 	record.erase(record.begin());
	// }
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_alignment_bias");
	ros::NodeHandle nh;

	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1, callback_imu);
	ros::Publisher pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/initial_pose", 1);
	ros::Publisher pub_bias = nh.advertise<sensor_msgs::Imu>("/imu_bias", 1);
	ros::Publisher pub_graph = nh.advertise<std_msgs::Float64>("/graphmsg", 1);

	X = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	P = 100.0*Eigen::MatrixXd::Identity(num_state, num_state);

	ros::Rate loop_rate(200);
	while(ros::ok())
	{
		double time = (ros::Time::now() - time_started).toSec();
		if(record.size()==0)	time = 0.0;
		const double timelimit = 180.0;
		if(imu_is_moving || time>timelimit){
			if(time>timelimit)	std::cout << "time > " << timelimit << "[s]" << std::endl;
			input_initialpose();
			break;
		}
		std_msgs::Float64 graphmsg;
		graphmsg.data = average.angular_velocity.y;
		pub_graph.publish(graphmsg);
		
		ros::spinOnce();

		loop_rate.sleep();
	}
	ros::Rate loop_rate_(1);
	while(ros::ok())
	{
		pub_inipose.publish(initial_pose);
		pub_bias.publish(average);
		loop_rate_.sleep();
	}
}
