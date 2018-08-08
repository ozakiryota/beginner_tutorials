/*
 * ekf_gyro_slam.cpp
 *
 * 状態：姿勢
 * 入力：ジャイロ	角速度	w_roll, w_pitach, w_yaw
 * 観測：SLAM		角度	roll, pitach, yaw
 *
 * X: |roll pitch yaw|^T	グローバル
 * U: |w_x w_y w_z|^T		ローカル
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

std::vector<IMUDATA> record;
sensor_msgs::Imu imu;
ros::Time current_time;
ros::Time last_time;
bool first_callback = true;

Eigen::MatrixXf X(3, 1);
// Eigen::MatrixXf U(3, 1);
Eigen::MatrixXf P(3, 3);
// Eigen::MatrixXf jF(3, 3);
// Eigen::MatrixXf A(3, 3);
// Eigen::MatrixXf B(3, 3);


geometry_msgs::Quaternion q;
const double LOOP_RATE = 100.1;

void strapdown(void)
{
	// float roll = atan2(imu.linear_acceleration.y, imu.linear_acceleration.z);
	// float pitch = atan2(-imu.linear_acceleration.x, sqrt(imu.linear_acceleration.y*imu.linear_acceleration.y + imu.linear_acceleration.z*imu.linear_acceleration.z));
	float roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
	
	Eigen::MatrixXf Z(2, 1);
	Z <<	atan2(imu.linear_acceleration.y, imu.linear_acceleration.z),
			atan2(-imu.linear_acceleration.x, sqrt(imu.linear_acceleration.y*imu.linear_acceleration.y + imu.linear_acceleration.z*imu.linear_acceleration.z));
	Eigen::MatrixXf H(2, 3);
	H <<	1,	0,	0,
			0,	1,	0;
	Eigen::MatrixXf jH(2, 3);
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

bool judge_stopping
{
	const float threshold = 0.001;
	if(fabs(record[record.size()-1].wx - ave.wx)>threshold)	return false;
	if(fabs(record[record.size()-1].wy - ave.wy)>threshold)	return false;
	if(fabs(record[record.size()-1].wz - ave.wz)>threshold)	return false;
	if(fabs(record[record.size()-1].ax - ave.ax)>threshold)	return false;
	if(fabs(record[record.size()-1].ay - ave.ay)>threshold)	return false;
	if(fabs(record[record.size()-1].az - ave.az)>threshold)	return false;
	std::cout << "non move" << std::endl;
	return true;
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "imu_callback" << std::endl;
	imu = *msg;

	IMUDATA ave = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i=0;i<record.size();i++){
		ave.wx += record[i].wx/(float)record.size();
		ave.wy += record[i].wy/(float)record.size();
		ave.wz += record[i].wz/(float)record.size();
		ave.ax += record[i].ax/(float)record.size();
		ave.ay += record[i].ay/(float)record.size();
		ave.az += record[i].az/(float)record.size();
	}
	record.push_back({
		imu.angular_velocity.x,
		imu.angular_velocity.y,
		imu.angular_velocity.z,
		imu.linear_acceleration.x,
		imu.linear_acceleration.y,
		imu.linear_acceleration.z
	});
	if(!first_callback){
		if(judge_stopping)	strapdown();
	}
		
	const size_t num_record = 100;
	if(record.size()>num_record)	record.erace(record.begin());


	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	if(!first_callback){
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
	}
	
	first_callback = false;
}

void callback_observation0(const geometry_msgs::QuaternionConstPtr& msg)
{
	tf::Quaternion q(msg->x, msg->y, msg->z, msg->w);
	double roll;
	double pitch;
	double yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	Eigen::MatrixXf H(3, 3);
	H <<	1,  0,  0,
			0,  1,  0;  
			0,  0,  1;

	Eigen::MatrixXf Z(3, 1);
	Z <<	roll,
			pitch,
			yaw;
	
	Eigen::MatrixXf I;
	I <<	1,  0,  0,
			0,  1;  0,
			0,  0,  1;
	
	Eigen::MatrixXf R(3, 3);

	Eigen::MatrixXf Y(3, 1);
	Eigen::MatrixXf S(3, 3);
	Eigen::MatrixXf K(3, 3);
	
	Y = Z - H*X;
	S = H*P*H.transpose();
	K = P*H.transpose()*S.inverse();
	X = X + (K*Y);
	P = (I - K*H)*P;
}

void callback_observation_usingwalls(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	const float g = 9.80665;
	Eigen::MatrixXf Z(2, 1);
	Z <<	atan2(msg->points[0].normal_y*g, msg->points[0].normal_z*g),
			atan2(-msg->points[0].normal_x*g, sqrt(msg->points[0].normal_y*g*msg->points[0].normal_y*g + msg->points[0].normal_z*g*msg->points[0].normal_z*g));
	Eigen::MatrixXf H(2, 3);
	H <<	1,	0,	0,
			0,	1,	0;
	Eigen::MatrixXf jH(2, 3);
	jH <<	1,	0,	0,
			0,	1,	0;
	Eigen::MatrixXf I(3, 3);
	I <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf R(3, 3);
	
	Eigen::MatrixXf Y(2, 1);
	Eigen::MatrixXf S(2, 2);
	Eigen::MatrixXf K(3, 2);
	
	Y = Z - H*X;
	S = jH*P*jH.transpose();
	K = P*H.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*H)*P;
}

void callback_observation_slam(const geometry::QuoternionConstPtr& msg)
{
	float roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
	
	Eigen::MatrixXf Z(3, 1);
	Z <<	roll,
			pitch,
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
	Eigen::MatrixXf R(3, 3);
	
	Eigen::MatrixXf Y(3, 1);
	Eigen::MatrixXf S(3, 3);
	Eigen::MatrixXf K(3, 3);
	
	Y = Z - H*X;
	S = jH*P*jH.transpose();
	K = P*H.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*H)*P;
}

void prediction0(Eigen::MatrixXf& X, Eigen::MatrixXf& P, float dt)
{
	float ang_x = X.coeffRef(0, 0);
	float ang_y = X.coeffRef(1, 0);
	float ang_z = X.coeffRef(2, 0);
	float w_xx = imu.angular_velocity.x;
	float w_yy = imu.angular_velocity.y;
	float w_zz = imu.angular_velocity.z;

	Eigen::MatrixXf A(3, 3);
	A <<	1,	0,	0,
	  		0,	1;	0,
			0,	0,	1;
	Eigen::MatrixXf B(3, 3);	//参照：https://hamachannel.hatenablog.com/entry/2018/05/13/155504
	B <<	1,	sin(ang_x)*tan(ang_y),	cos(ang_x)*tan(ang_y),
			0,	cos(ang_x),		-sin(ang_x),
			0,	sin(ang_x)/cos(ang_y),	cos(ang_x)/cos(ang_y);
	Eigen::MatrixXf U(3, 1);
	U <<	w_xx*dt,
	  	w_yy*dt,
		w_zz*dt;
	Eigen::MatrixXf jF(3, 3);	//ヤコビ行列
	jF <<	1+dt*(w_yy*tan(ang_y)*cos(ang_x)-w_zz*tan(ang_y)*sin(ang_x)),	dt*(-w_yy*sin(ang_x)-w_zz*cos(ang_x)),	dt*(w_yy/cos(ang_y)*cos(ang_x)-w_zz/cos(ang_y)*sin(ang_x)),
		dt*(w_yy*sin(ang_x)/(cos(ang_y)*cos(ang_y))+w_zz*cos(ang_x)/(cos(ang_y)*cos(ang_y))),	1,	dt*(w_yy*sin(ang_x)*sin(ang_y)/(cos(ang_y)*cos(ang_y))+w_zz*cos(ang_x)*sin(ang_y)/(cos(ang_y)*cos(ang_y))),
		0,	0,	1;	
	X = A*X+B*U;
	P = jF*P*jF.transpose();
}

void mesurement_update(Eigen::MatrixXf& X, Eigen::MatrixXf& P, float dt, float a_x, float a_y, float a_z)
{
	//SLAMから得られる姿勢
	float roll;
	float pitch;
	float yaw;

	//const int n = 4;
	Eigen::MatrixXf H(3, 3);
	H <<	1,	0,	0,
	  		0,	1,	0;
	  		0,	0,	1;
	Eigen::MatrixXf Z(3, 1);
	Z <<	roll,
			pitch,
			yaw;
	Eigen::MatrixXf Y(3, 1);
	Eigen::MatrixXf S(3, 3);
	Eigen::MatrixXf R(3, 3);
	R <<	0.03,	0,	0,
	  		0,	0.03,	0,
			0,	0,	0.03;
	Eigen::MatrixXf K(3, 3);
	Eigen::MatrixXf I;
	I <<	1,	0,	0,
	  		0,	1;	0,
			0,	0,	1;

	Y = Z-H*X;
	S = H*P*H.transpose()+R;
	K = P*H.transpose()*S.inverse();
	X = X+(K*Y);
	P = (I-K*H)*P;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Subscriber sub_obs = nh.subscribe("/g_usingwalls", 10, callback_observation_usingwalls);

	Eigen::MatrixXf X = Eigen::MatrixXf::Zero(3, 1);
	Eigen::MatrixXf P(3, 3);
	P <<	1000,	0,	0,
	  		0,	 1000,	0,
			0,	0,	1000;
	X <<	0.0,
	  		0.0,
			0.0;

	while(ros::ok()){
		ros::spinOnce();
	}
}
