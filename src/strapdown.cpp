/*
 *	strapdown.cpp
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

struct IMUDATA{
	float wx;
	float wy;
	float wz;
	float ax;
	float ay;
	float az;
};

sensor_msgs::Imu imu;
geometry_msgs::Quaternion ini_pose;
std::vector<IMUDATA> record;
IMUDATA ave;
ros::Time current_time;
ros::Time last_time;
// bool first_callback = true;
bool moving = false;
Eigen::MatrixXf X(3, 1);
Eigen::MatrixXf P(3, 3);

void strapdown(void)
{
	Eigen::MatrixXf Z(2, 1);
	Z <<	atan2(imu.linear_acceleration.y, imu.linear_acceleration.z),
			atan2(-imu.linear_acceleration.x, sqrt(imu.linear_acceleration.y*imu.linear_acceleration.y + imu.linear_acceleration.z*imu.linear_acceleration.z));
	Eigen::MatrixXf H(2, 3);
	H <<	1,	0,	0,
			0,	1,	0;
	Eigen::MatrixXf jH(2, 1);
	jH <<	1,	0,	0,
			0,	1,	0;
	Eigen::MatrixXf I(3, 3);
	I <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf Y(2, 1);
	Eigen::MatrixXf S(2, 2);
	Eigen::MatrixXf K(3, 2);
	
	Y = Z - H*X;
	S = jH*P*jH.transpose();
	K = P*H.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*H)*P;
}

void strapdown1(void)
{
	float roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
	
	Eigen::MatrixXf Z(3, 1);
	Z <<	atan2(ave.ay, ave.az),
			atan2(-ave.ax, sqrt(ave.ay*ave.ay + ave.az*ave.az)),
			yaw;
	Eigen::MatrixXf H(3, 3);
	H <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf jH(3, 3);
	jH <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf I(3, 3);
	I <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf Y(3, 1);
	Eigen::MatrixXf S(3, 3);
	Eigen::MatrixXf K(3, 3);
	
	Y = Z - H*X;
	S = jH*P*jH.transpose();
	K = P*H.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*H)*P;
}

bool judge_moving(void)
{
	const float threshold = 0.001;
	if(fabs(record[record.size()-1].wx - ave.wx)>threshold)	return true;
	if(fabs(record[record.size()-1].wy - ave.wy)>threshold)	return true;
	if(fabs(record[record.size()-1].wz - ave.wz)>threshold)	return true;
	if(fabs(record[record.size()-1].ax - ave.ax)>threshold)	return true;
	if(fabs(record[record.size()-1].ay - ave.ay)>threshold)	return true;
	if(fabs(record[record.size()-1].az - ave.az)>threshold)	return true;
	std::cout << "non move" << std::endl;
	return false;
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "imu_callback" << std::endl;
	imu = *msg;
	
	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	record.push_back({
		imu.angular_velocity.x,
		imu.angular_velocity.y,
		imu.angular_velocity.z,
		imu.linear_acceleration.x,
		imu.linear_acceleration.y,
		imu.linear_acceleration.z
	});
	const size_t num_record = 100;
	if(record.size()>num_record)	record.erace(record.begin());
	
	ave = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i=0;i<record.size();i++){
		ave.wx += record[i].wx/(float)record.size();
		ave.wy += record[i].wy/(float)record.size();
		ave.wz += record[i].wz/(float)record.size();
		ave.ax += record[i].ax/(float)record.size();
		ave.ay += record[i].ay/(float)record.size();
		ave.az += record[i].az/(float)record.size();
	}
	
	if(record.size()>num_record)	moving = judge_moving();
	if(moving){
		std::cout << "moved" << std::endl;
		tf::Quaternion tmp_q = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
		quaternionTFToMsg(tmp_q, ini_pose);
	}

	if(record.size()>num_record&&!moving){
		Eigen::MatrixXf U(3, 1);
		U <<	-imu.angular_velocity.x*dt,
		  		-imu.angular_velocity.y*dt,
				-imu.angular_velocity.z*dt;

		Eigen::MatrixXf A(3, 3);
		A <<	1,  0,  0,
				0,  1,  0,
				0,  0,  1;

		Eigen::MatrixXf B(3, 3);
		B <<    1,	sin(X(0, 0))*tan(X(1, 0)),	cos(X(0, 0))*tan(X(1, 0)),
	    		0,	cos(X(0, 0)),	-sin(X(1, 0)),
				0,	sin(X(0, 0))/cos(X(1, 0)),	cos(X(0, 0))/cos(X(1, 0));

		Eigen::MatrixXf jF(3, 3);
		jF <<	U(1, 0)*tan(X(1, 0))*cos(X(0, 0)) - U(2, 0)*tan(X(1, 0))*sin(X(0, 0)),
					U(1, 0)*sin(X(0, 0))/( cos(X(1, 0))*cos(X(1, 0)) ) - U(2, 0)*cos(X(0, 0))/( cos(X(1, 0))*cos(X(1, 0)) ),
						0,
				-U(1, 0)*sin(X(0, 0)) - U(2, 0)*cos(X(0, 0)),
					0,
						0,
				U(1, 0)/cos(X(1, 0))*cos(X(0, 0)) - U(2, 0)/cos(X(1, 0))*sin(X(0, 0)),
					U(1, 0)*sin(X(0, 0 ))*sin(X(1, 0))/( cos(X(1, 0))*cos(X(1, 0)) ) + U(2, 0)*cos(X(0, 0))*sin(X(1, 0))/( cos(X(1, 0))*cos(X(1, 0)) ),
						0;
		
		Eigen::MatrixXf Q(3, 3);
		
		X = A*X + B*U;
		P = jF*P*jF.transpose();
		
		
		strapdown1();
	}
	
	// first_callback = false;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Publisher pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/ini_pose",1);

	Eigen::MatrixXf X(3, 1);
	X <<	0.0,
	  		0.0,
			0.0;
	Eigen::MatrixXf P(3, 3);
	P <<	1000,	0,	0,
	  		0,	 1000,	0,
			0,	0,	1000;
	

	while(ros::ok()&&!moving){
		ros::spinOnce();
	}
	
	ros::Rate loop_rate(10);
	while(ros::ok()){
		pub_inipose.publish(ini_pose);
		loop_rate.sleep();
	}
}