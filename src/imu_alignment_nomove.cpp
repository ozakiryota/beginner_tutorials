/*
 *	imu_alignment_nomove.cpp
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include "beginner_tutorials/imudata.h"

// struct IMUDATA{
// 	double wx;
// 	double wy;
// 	double wz;
// 	double ax;
// 	double ay;
// 	double az;
// 	double yaw;
// };

sensor_msgs::Imu imu;
geometry_msgs::Quaternion ini_pose;
// std::vector<IMUDATA> record;
// IMUDATA ave;
// std::vector<double> bias(6, 0.0);
std::vector<beginner_tutorials::imudata> record;
beginner_tutorials::imudata ave;
beginner_tutorials::imudata bias;
bool imu_is_moving = false;
// FILE* fp;

void calculate_inipose(void)
{
	double roll, pitch, yaw;
	
	roll = atan2(ave.ay, ave.az);
	pitch = atan2(-ave.ax, sqrt(ave.ay*ave.ay + ave.az*ave.az));
	yaw = ave.yaw;
	yaw = 0.0;
	
	tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
	quaternionTFToMsg(q, ini_pose);
}

void rotation(geometry_msgs::Quaternion q, Eigen::MatrixXf& X, bool inverse)
{
	if(inverse) q.w *= -1; 
	Eigen::MatrixXf R(3, 3); 
	R <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,	2*(q.x*q.y - q.w*q.z),	2*(q.x*q.z + q.w*q.y),
			2*(q.x*q.y + q.w*q.z),	q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,	2*(q.y*q.z - q.w*q.x),
			2*(q.x*q.z - q.w*q.y),	2*(q.y*q.z + q.w*q.x),	q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	X = R*X;
	// std::cout << "X = " << std::endl << X << std::endl;
}

void calculate_bias(void)
{
	tf::Quaternion tmp_q(ini_pose.x, ini_pose.y, ini_pose.z, ini_pose.w);
	// Eigen::MatrixXf R(3, 3);
	// R <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,	2*(q.x*q.y - q.w*q.z),	2*(q.x*q.z + q.w*q.y),
	// 		2*(q.x*q.y + q.w*q.z),	q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,	2*(q.y*q.z - q.w*q.x),
	// 		2*(q.x*q.z - q.w*q.y),	2*(q.y*q.z + q.w*q.x),	q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	Eigen::MatrixXf G(3, 1);
	G <<	0,
			0,
			9.80665;
	rotation(ini_pose, G, false);
	std::cout << "G = " << std::endl << G << std::endl;
	
	bias.wx = ave.wx;
	bias.wy = ave.wy;
	bias.wz = ave.wz;
	bias.ax = ave.ax - G(0, 0);
	bias.ay = ave.ay - G(1, 0);
	bias.az = ave.az - G(2, 0);
	std::cout << "bias = " << std::endl << bias << std::endl;
	
	FILE* fp;
	fp = fopen("/home/amsl/Desktop/imudata_nomove.csv", "w");
	for(size_t i=0;i<record.size();i++)	fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", i,record[i].wx, record[i].wy, record[i].wz ,record[i].ax, record[i].ay, record[i].az, record[i].yaw);
	for(size_t i=0;i<record.size();i++)	fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%f\t%f\n", i,record[i].wx, record[i].wy, record[i].wz ,record[i].ax-G(0,0), record[i].ay-G(1,0), record[i].az-G(2,0));
	fclose(fp);
}

bool judge_moving(void)
{
	const float threshold_w = 0.03;
	const float threshold_a = 0.03;
	const float threshold_yaw = 0.01;
	// std::cout << "record[record.size()-1].wx - ave.wx = " << record[record.size()-1].wx - ave.wx << std::endl;
	
	// if(fabs(record[record.size()-1].wx - record[record.size()-2].wx)>threshold_w)	return true;
	// if(fabs(record[record.size()-1].wy - record[record.size()-2].wy)>threshold_w)	return true;
	// if(fabs(record[record.size()-1].wz - record[record.size()-2].wz)>threshold_w)	return true;
	// if(fabs(record[record.size()-1].ax - record[record.size()-2].ax)>threshold_a)	return true;
	// if(fabs(record[record.size()-1].ay - record[record.size()-2].ay)>threshold_a)	return true;
	// if(fabs(record[record.size()-1].az - record[record.size()-2].az)>threshold_a)	return true;
	// if(fabs(record[record.size()-1].yaw - record[record.size()-2].yaw)>threshold_yaw)	return true;
	if(fabs(record[record.size()-1].wx - ave.wx)>threshold_w)	return true;
	if(fabs(record[record.size()-1].wy - ave.wy)>threshold_w)	return true;
	if(fabs(record[record.size()-1].wz - ave.wz)>threshold_w)	return true;
	if(fabs(record[record.size()-1].ax - ave.ax)>threshold_a)	return true;
	if(fabs(record[record.size()-1].ay - ave.ay)>threshold_a)	return true;
	if(fabs(record[record.size()-1].az - ave.az)>threshold_a)	return true;
	if(fabs(record[record.size()-1].yaw - ave.yaw)>threshold_yaw)	return true;
	
	// std::cout << "non move" << std::endl;
	return false;
}

void calculate_average(void)
{
	// ave = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	ave.wx = 0.0;
	ave.wy = 0.0;
	ave.wz = 0.0;
	ave.ax = 0.0;
	ave.ay = 0.0;
	ave.az = 0.0;
	ave.yaw = 0.0;
	
	for(size_t i=0;i<record.size();i++){
		ave.wx += record[i].wx/(float)record.size();
		ave.wy += record[i].wy/(float)record.size();
		ave.wz += record[i].wz/(float)record.size();
		ave.ax += record[i].ax/(float)record.size();
		ave.ay += record[i].ay/(float)record.size();
		ave.az += record[i].az/(float)record.size();
		ave.yaw += record[i].yaw/(float)record.size();
	}
	// std::cout << "ave.wx = " << ave.wx << std::endl;
	// std::cout << "ave.wy = " << ave.wy << std::endl;
	// std::cout << "ave.wz = " << ave.wz << std::endl;
	// std::cout << "ave.ax = " << ave.ax << std::endl;
	// std::cout << "ave.ay = " << ave.ay << std::endl;
	// std::cout << "ave.az = " << ave.az << std::endl;
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "imu_callback" << std::endl;
	imu = *msg;
	
	double roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);

	beginner_tutorials::imudata tmp;
	tmp.wx = imu.angular_velocity.x;
	tmp.wy = imu.angular_velocity.y;
	tmp.wz = imu.angular_velocity.z;
	tmp.ax = imu.linear_acceleration.x;
	tmp.ay = imu.linear_acceleration.y;
	tmp.az = imu.linear_acceleration.z;
	tmp.yaw = yaw;
	record.push_back(tmp);
	
	// fprintf(fp, "%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", record.size(), tmp.wx, tmp.wy, tmp.wz ,tmp.ax, tmp.ay, tmp.az, tmp.yaw);
	// record.push_back({
	// 	imu.angular_velocity.x,
	// 	imu.angular_velocity.y,
	// 	imu.angular_velocity.z,
	// 	imu.linear_acceleration.x,
	// 	imu.linear_acceleration.y,
	// 	imu.linear_acceleration.z,
	// 	roll,
	// 	pitch,
	// 	yaw
	// });
	
	calculate_average();
	if(record.size()>1)	imu_is_moving = judge_moving();
	
	if(imu_is_moving){
		// calculate_average();
		calculate_inipose();
		calculate_bias();
	}
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_alignment_nomove");
	ros::NodeHandle nh;

	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Publisher pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/initial_pose", 1);
	ros::Publisher pub_bias = nh.advertise<beginner_tutorials::imudata>("/imu_bias", 1);	

	// fp = fopen("/home/amsl/Desktop/imudata.csv", "w");
	
	while(ros::ok()&&!imu_is_moving){
		ros::spinOnce();
	}
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		// ros::spinOnce();
		pub_inipose.publish(ini_pose);
		pub_bias.publish(bias);
		loop_rate.sleep();
	}
	// fclose(fp);
}
