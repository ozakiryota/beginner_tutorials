/*
 * ekf.cpp
*/

#include <ros/ros.h>
#include <Eigen/Core>

void prediction(float dt)
{
	/*
	 * X: |w_x, w_y, w_z, a_x, a_y, a_z|^T
	 * u: |v, w|
	 */
	Eigen::MatrixXf F;
	// F <<	
	Eigen::MatrixXf _F;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;
}
