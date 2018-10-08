#include<ros/ros.hi>
#include <tf/tf.h>

int main(int argc, char**argv)
{
	ros::init(argc, argv, "ekf");
	ros::NodeHandle nh;
	
	while(ros::ok()){
		double x, y, z, w;
		std::cout << "x = ?" << std::endl;
		scanf("%lf", &x);
		std::cout << "y = ?" << std::endl;
		scanf("%lf", &y);
		std::cout << "z = ?" << std::endl;
		scanf("%lf", &z);
		std::cout << "w = ?" << std::endl;
		scanf("%lf", &w);

		tf::Quaternion q(x, y, z, w);
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		std::cout << "roll = " << roll << std::endl;
		std::cout << "pitch = " << pitch << std::endl;
		std::cout << "yaw = " << yaw << std::endl;
	}
}
