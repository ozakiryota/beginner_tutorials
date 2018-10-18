#include<ros/ros.h>
#include <tf/tf.h>

int main(int argc, char**argv)
{
	ros::init(argc, argv, "quaternion_convertor");
	ros::NodeHandle nh;
	
	while(ros::ok()){
		double x, y, z, w;
		std::cout << "q.x = ?" << std::endl;
		scanf("%lf", &x);
		std::cout << "q.y = ?" << std::endl;
		scanf("%lf", &y);
		std::cout << "q.z = ?" << std::endl;
		scanf("%lf", &z);
		std::cout << "q.w = ?" << std::endl;
		scanf("%lf", &w);

		// x = -0.00274389517984;
		// y = 0.00524909114424;
		// z = 0.0645202091611;
		// w = 0.997898822873;

		tf::Quaternion q(x, y, z, w);
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		std::cout << "roll  = " << roll << "[rad]   " << roll/M_PI*180.0 << "[deg]" << std::endl;
		std::cout << "pitch = " << pitch << "[rad]   " << pitch/M_PI*180.0 << "[deg]" << std::endl;
		std::cout << "yaw   = " << yaw << "[rad]   " << yaw/M_PI*180.0 << "[deg]" << std::endl;
	}
}
