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
 
sensor_msgs::Imu imu;
tf::Quaternion q_imu(0.0, 0.0, 0.0, 1.0);
const double LOOP_RATE = 100.1;

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
	std::cout << "imu_callback" << std::endl;
	imu = *msg;

	double roll = -imu.angular_velocity.x*(1/LOOP_RATE);
	double pitch = -imu.angular_velocity.y*(1/LOOP_RATE);
	double yaw = -imu.angular_velocity.z*(1/LOOP_RATE);
	q_imu = tf::createQuaternionFromRPY(roll, pitch, yaw)*q_imu;
}

void observation_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{

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

	Eigen::MatrixXf X = Eigen::MatrixXf::Zero(3, 1);
	Eigen::MatrixXf P(3, 3);
	P <<	1000,	0,	0,
	  		0,	 1000,	0,
			0,	0,	1000;
}
