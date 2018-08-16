#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
// #include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "beginner_tutorials/imudata.h"

struct IMUDATA{
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
};

geometry_msgs::Quaternion initial_pose;
sensor_msgs::Imu imu;
sensor_msgs::Imu imu_last;
nav_msgs::Odometry odom;
bool inipose_is_available = false;
Eigen::MatrixXf V(3, 1);
std::string PARENT_FRAME;
ros::Time current_time;
ros::Time last_time;
// std::vector<beginner_tutorials::imudata> record;
// IMUDATA bias_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
beginner_tutorials::imudata bias;
FILE* fp;

void rotation(geometry_msgs::Quaternion q,	Eigen::MatrixXf& X, bool inverse)
{
	if(inverse)	q.w *= -1;
	Eigen::MatrixXf R(3, 3);
	R <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,	2*(q.x*q.y - q.w*q.z),	2*(q.x*q.z + q.w*q.y),
			2*(q.x*q.y + q.w*q.z),	q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,	2*(q.y*q.z - q.w*q.x),
			2*(q.x*q.z - q.w*q.y),	2*(q.y*q.z + q.w*q.x),	q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	X = R*X;
	// std::cout << "X = " << std::endl << X << std::endl;
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "CALLBACK IMU" << std::endl;
	imu = *msg;

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;
	// std::cout << "dt = " << dt << std::endl;
	
	// if(!inipose_is_available)	record.push_back({
	// 								imu.angular_velocity.x,
	// 								imu.angular_velocity.y,
	// 								imu.angular_velocity.z,
	// 								imu.linear_acceleration.x,
	// 								imu.linear_acceleration.y,
	// 								imu.linear_acceleration.z
	// 							});

	if(inipose_is_available){
		// record.erase(record.begin());
		// bias = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		// for(size_t i=0;i<record.size();i++){
		// 	bias.wx += record[i].wx/(double)record.size();
		// 	bias.wy += record[i].wy/(double)record.size();
		// 	bias.wz += record[i].wz/(double)record.size();
		// 	bias.ax += record[i].ax/(double)record.size();
		// 	bias.ay += record[i].ay/(double)record.size();
		// 	bias.az += record[i].az/(double)record.size();
		// }
		// std::cout << "bias.wx = " << bias.wx << std::endl;
		// std::cout << "bias.wy = " << bias.wy << std::endl;
		// std::cout << "bias.wz = " << bias.wz << std::endl;
		// std::cout << "bias.ax = " << bias.ax << std::endl;
		// std::cout << "bias.ay = " << bias.ay << std::endl;
		// std::cout << "bias.az = " << bias.az << std::endl;


		tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		double wx = imu.angular_velocity.x*dt;
		double wy = imu.angular_velocity.y*dt;
		double wz = imu.angular_velocity.z*dt;
		q = tf::createQuaternionFromRPY(wx, wy, wz)*q;
		quaternionTFToMsg(q, odom.pose.pose.orientation);
		
		// double roll, pitch, yaw;
		// tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		// fprintf(fp, "%f\t%f\t%f\n", roll, pitch, yaw);
		
		Eigen::MatrixXf Acc(3, 1);
		Acc <<	-imu.linear_acceleration.x,
				-imu.linear_acceleration.y,
				-imu.linear_acceleration.z;
		// Acc <<	-imu.linear_acceleration.x + bias.ax,
		// 		-imu.linear_acceleration.y + bias.ay,
		// 		-imu.linear_acceleration.z + bias.az;
			
		// const double a = 0.5;	//Low Pass Filter 
		// Acc <<	-(a*imu.linear_acceleration.x + (1.0 - a)*imu_last.linear_acceleration.x),
		// 		-(a*imu.linear_acceleration.y + (1.0 - a)*imu_last.linear_acceleration.y),
		// 		-(a*imu.linear_acceleration.z + (1.0 - a)*imu_last.linear_acceleration.z);

		// Eigen::MatrixXf R_(3, 3);
		// R_ <<	cos(roll)*cos(pitch),
		//   			cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
		// 					cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
		// 			sin(roll)*cos(pitch),
		//   			sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
		// 					 sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
		// 			-sin(pitch),
		// 				cos(pitch)*sin(yaw),
		// 					cos(pitch)*cos(yaw);
		// Eigen::MatrixXf Rot(3, 3);
		// Rot <<	cos(pitch)*cos(yaw),
		// 			cos(pitch)*sin(yaw),
		// 				-sin(pitch),
		// 		sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
		// 			sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
		// 				sin(roll)*cos(pitch),
		// 		cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
		// 			cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
		// 				cos(roll)*cos(pitch);
		
		// Rot <<	cos(roll)*cos(pitch)*cos(yaw) - sin(roll)*sin(yaw),
		// 			-cos(roll)*cos(pitch)*sin(yaw) - sin(roll)*cos(yaw),
		// 				cos(roll)*sin(pitch),
		// 		sin(roll)*cos(pitch)*cos(yaw) + cos(roll)*sin(yaw),
		// 			-sin(roll)*cos(pitch)*sin(yaw) + cos(roll)*cos(yaw),
		//   				sin(roll)*sin(pitch),
		// 		-sin(pitch)*cos(yaw),
		// 			sin(pitch)*sin(yaw),
		// 				cos(pitch);

		// std::cout << "R = " << std::endl << R << std::endl;
		// std::cout << "R.inverse() = " << std::endl << R.inverse() << std::endl;
		Eigen::MatrixXf G(3, 1);
		G <<	0.0,
				0.0,
				-9.80665;
		std::cout << "Acc_before = " << std::endl << Acc << std::endl;
		rotation(odom.pose.pose.orientation, Acc, true);
		V = V + (Acc - G)*dt;
		// V = V + (Rot.inverse()*Acc - G)*dt;
		std::cout << "Acc_after = " << std::endl << Acc << std::endl;
		std::cout << "V = " << std::endl << V << std::endl;
		// odom.twist.twist.linear.x = V(0, 0);
		// odom.twist.twist.linear.y = V(1, 0);
		// odom.twist.twist.linear.z = V(2, 0);
		odom.pose.pose.position.x += V(0, 0)*dt;
		odom.pose.pose.position.y += V(1, 0)*dt;
		odom.pose.pose.position.z += V(2, 0)*dt;
		// odom.pose.pose.position.x += V(0, 0)*dt - bias.ax;
		// odom.pose.pose.position.y += V(1, 0)*dt - bias.ay;
		// odom.pose.pose.position.z += V(2, 0)*dt - bias.az;
		
		fprintf(fp, "%f\t%f\t%f\n", -imu.linear_acceleration.x, Acc(0, 0), V(0, 0));

		static tf::TransformBroadcaster broadcaster;
		geometry_msgs::TransformStamped transform;
		transform.header.stamp = ros::Time::now();
		transform.header.frame_id = PARENT_FRAME;
		transform.child_frame_id = "/imu_";
		transform.transform.translation.x = odom.pose.pose.position.x;
		transform.transform.translation.y = odom.pose.pose.position.y;
		transform.transform.translation.z = odom.pose.pose.position.z;
		// transform.transform.translation.x = 0.0;
		// transform.transform.translation.y = 0.0;
		// transform.transform.translation.z = 0.0;
		transform.transform.rotation = odom.pose.pose.orientation;
		broadcaster.sendTransform(transform);
	}
	imu_last = imu;
}

void callback_bias(const beginner_tutorials::imudataConstPtr& msg)
{
	// std::cout << "CALLBACK BIAS" << std::endl;
	bias = *msg;
	std::cout << "bias = " << std::endl << bias << std::endl;
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	// initial_pose = *msg;
	if(!inipose_is_available){
		initial_pose = *msg;

		odom.header.frame_id = PARENT_FRAME;
		odom.child_frame_id = "/imu_";
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = initial_pose;
		// odom.pose.pose.orientation = imu.orientation;	//(仮)
		// odom.pose.pose.orientation.x = 0.0;	//(仮)
		// odom.pose.pose.orientation.y = 0.0;	//(仮)
		// odom.pose.pose.orientation.z = 0.0;	//(仮)
		// odom.pose.pose.orientation.w = 1.0;	//(仮)

		
		// for(size_t i=0;i<record.size();i++){
		// 	bias.wx += record[i].wx/(double)record.size();
		// 	bias.wy += record[i].wy/(double)record.size();
		// 	bias.wz += record[i].wz/(double)record.size();
		// 	bias.ax += record[i].ax/(double)record.size();
		// 	bias.ay += record[i].ay/(double)record.size();
		// 	bias.az += record[i].az/(double)record.size();
		// }

		// tf::Quaternion q_(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);		
		// double roll, pitch, yaw;
		// tf::Matrix3x3(q_).getRPY(roll, pitch, yaw);

		// Eigen::MatrixXf G(3, 1);
		// G <<	0.0,
		//   		0.0,
		// 		9.80665;
		
		// Eigen::MatrixXf R_(3, 3);
		// R_ <<	cos(roll)*cos(pitch),
		//   			cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw),
		// 					cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw),
		// 			sin(roll)*cos(pitch),
		//   			sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw),
		// 					 sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw),
		// 			-sin(pitch),
		// 				cos(pitch)*sin(yaw),
		// 					cos(pitch)*cos(yaw);
		// rotation(odom.pose.pose.orientation, G, false);

		// Eigen::MatrixXf R__(3, 3);
		// R__ <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,	2*(q.x*q.y - q.w*q.z),	2*(q.x*q.z + q.w*q.y),
		// 		2*(q.x*q.y + q.w*q.z),	q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,	2*(q.y*q.z - q.w*q.x),
		// 		2*(q.x*q.z - q.w*q.y),	2*(q.y*q.z + q.w*q.x),	q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;

		// Eigen::MatrixXf R(3, 3);
		// R <<	cos(roll)*cos(pitch)*cos(yaw) - sin(roll)*sin(yaw),
		// 			-cos(roll)*cos(pitch)*sin(yaw) - sin(roll)*cos(yaw),
		// 				cos(roll)*sin(pitch),
		// 		sin(roll)*cos(pitch)*cos(yaw) + cos(roll)*sin(yaw),
		// 			-sin(roll)*cos(pitch)*sin(yaw) + cos(roll)*cos(yaw),
		//   				sin(roll)*sin(pitch),
		// 		-sin(pitch)*cos(yaw),
		// 			sin(pitch)*sin(yaw),
		// 				cos(pitch);
		// G = R__*G;
		// std::cout << "G = " << std::endl << G << std::endl;
		
		// for(size_t i=0;i<record.size();i++){
		// 	bias.wx += record[i].wx/(double)record.size();
		// 	bias.wy += record[i].wy/(double)record.size();
		// 	bias.wz += record[i].wz/(double)record.size();
		// 	bias.ax += (record[i].ax - G(0, 0))/(double)record.size();
		// 	bias.ay += (record[i].ay - G(1, 0))/(double)record.size();
		// 	bias.az += (record[i].az - G(2, 0))/(double)record.size();
		// }
        //
		// std::cout << "bias.wx = " << bias.wx << std::endl;
		// std::cout << "bias.wy = " << bias.wy << std::endl;
		// std::cout << "bias.wz = " << bias.wz << std::endl;
		// std::cout << "bias.ax = " << bias.ax << std::endl;
		// std::cout << "bias.ay = " << bias.ay << std::endl;
		// std::cout << "bias.az = " << bias.az << std::endl;
		inipose_is_available = true;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "imu_odom");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	local_nh.getParam("PARENT_FRAME", PARENT_FRAME);
	
	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	ros::Subscriber sub_bias = nh.subscribe("/imu_bias", 1, callback_bias);
	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1, callback_imu);
	ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/imu_odom", 1);

	current_time = ros::Time::now();
	last_time = ros::Time::now();
	V <<	0.0,
			0.0,
			0.0;

	fp = fopen("/home/amsl/Desktop/imudata.csv", "w");

	// ros::Rate loop_rate(10);
	while(ros::ok()){
		odom.header.stamp = ros::Time::now();
		if(inipose_is_available)	pub_odom.publish(odom);
		ros::spinOnce();
		// loop_rate.sleep();
	}
	fclose(fp);
}
