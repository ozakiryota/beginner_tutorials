/*
 * kf.cpp
 *
 * IMU用
 * ジャイロ(roll, pitch)のドリフトを
 * 加速度センサからの角度算出値を用いて補正
 *
 * 加速度センサーそのものが動的加速度を伴っている場合、正確な計算はできない
*/

#include <ros/ros.h>
#include <Eigen/Core>

void prediction(Eigen::MatrixXf& X, Eigen::MatrixXf& P, float dt)
{
	/*
	 * X: |roll[rad] pitch[rad] w_roll[rad/s] w_pitch[rad/s]|^T
	 * u: none
	 */
	Eigen::MatrixXf F(4, 4);
	F <<	1,	0,	dt,	0,
	  		0,	1;	0,	dt,
			0,	0,	1,	0,
			0,	0,	0,	1;
	//Eigen::MatrixXf _F;	//ヤコビ行列
	X = F*X;
	P = F*P*F.transpose();
}

void mesurement_update(Eigen::MatrixXf& X, Eigen::MatrixXf& P, float dt, float a_x, float a_y, float a_z)
{
	float roll_from_acc = X.coeffRef(0, 0)+atan2(a_z, a_y);
	float pitch_from_acc = X.coeffRef(1, 0)-atan2(sqrt(a_y*a_y+a_z*a_z), a_x);

	Eigen::MatrixXf H(2, 4);
	H <<	1,	0,	0,	0,
	  		0,	1;	0,	0;

	Eigen::MatrixXf Z(2, 0);
	Z <<	roll_from_acc,
			pitch_from_acc;
	Eigen::MatrixXf Y = Eigen::MatrixXf::Zero(4, 4);

	Y = Z-H*X;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;

	Eigen::MatrixXf X = Eigen::MatrixXf::Zero(4, 1);
	Eigen::MatrixXf P(4, 4);
	P <<	1000,	0,	0,	0,
	  		0,	 1000,	0,	0,
			0,	0,	1000,	0,
			0,	0,	0,	1000;
}
