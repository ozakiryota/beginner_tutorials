/*
	test_eigen.cpp
*/
#include <ros/ros.h>
#include <Eigen/Core>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_eigen");
	ros::NodeHandle nh;
	std::cout << "Learning Eigen" << std::endl;

	Eigen::MatrixXf A(2, 2);
	A <<	1,	2,
	  		3,	4;
	Eigen::MatrixXf B(2, 2);
	B <<	5,	6,
	  		7,	9;
	Eigen::MatrixXf C(2, 2);
	C = A*B;
	// std::cout << C << std::endl;

	Eigen::MatrixXd D = Eigen::MatrixXd::Random(2, 2);
	Eigen::MatrixXd E = Eigen::MatrixXd::Random(2, 1);
	Eigen::MatrixXd F = Eigen::MatrixXd::Random(1, 1);
	std::cout << "D" <<  std::endl << D << std::endl;
	std::cout << "E" <<  std::endl << E << std::endl;
	// std::cout << "D*E" <<  std::endl << D*E << std::endl;
	// std::cout << "D+E" <<  std::endl << D+E << std::endl;
	std::cout << "D*E*F" <<  std::endl << D*E*F << std::endl;

	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
	std::cout << "I" <<  std::endl << I << std::endl;
}
