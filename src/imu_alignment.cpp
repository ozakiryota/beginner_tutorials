/*
 *	imu_alignment.cpp.cpp
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

struct IMUDATA{
	double wx;
	double wy;
	double wz;
	double ax;
	double ay;
	double az;
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

void strapdown1(void)
{
	double roll, pitch, yaw;
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
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;

	std::cout << "~ observation ~" << std::endl;
	std::cout << "Z = " << std::endl << Z << std::endl;
	std::cout << "Z - H = Y = " << std::endl << Y << std::endl;
	std::cout << "jH*P*jH.transpose() = S = " << std::endl << S << std::endl;
	std::cout << "S.determinant() = " << S.determinant() << std::endl;
	std::cout << "S.inverse() = " << std::endl << S.inverse() << std::endl;
	std::cout << "P*jH.transpose()*S.inverse() = K = " << std::endl << K << std::endl;
	std::cout << "X + K*Y = X = " << std::endl << X << std::endl;
	std::cout << "(I - K*jH)*P = P = " << std::endl << P << std::endl;
}

void strapdown2(void)
{
	double roll, pitch, yaw;
	tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
	
	Eigen::MatrixXf Z(3, 1);
	Z <<	imu.linear_acceleration.x,
	  		imu.linear_acceleration.y,
			imu.linear_acceleration.z;
	const float g = 9.80665;
	Eigen::MatrixXf H(3, 1);
	H <<	g*sin(X(1, 0)),
			-g*sin(X(0, 0))*cos(X(1, 0)),
			-g*cos(X(0, 0))*cos(X(1, 0));
	Eigen::MatrixXf jH(3, 3);
	jH <<	0,	g*cos(X(1, 0)),	0,
			-g*cos(X(0, 0))*cos(X(1, 0)),	g*sin(X(0, 0))*sin(X(1, 0)),	0,
			g*sin(X(0, 0))*cos(X(1, 0)),	g*cos(X(0, 0))*sin(X(1, 0)),	0;
	Eigen::MatrixXf I(3, 3);
	I <<	1,	0,	0,
			0,	1,	0,
			0,	0,	1;
	Eigen::MatrixXf Y(3, 1);
	Eigen::MatrixXf S(3, 3);
	Eigen::MatrixXf K(3, 3);
	
	Y = Z - H;
	S = jH*P*jH.transpose();
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	P = (I - K*jH)*P;

	std::cout << "~ observation ~" << std::endl;
	std::cout << "Z = " << std::endl << Z << std::endl;
	std::cout << "Z - H = Y = " << std::endl << Y << std::endl;
	std::cout << "jH*P*jH.transpose() = S = " << std::endl << S << std::endl;
	std::cout << "S.determinant() = " << S.determinant() << std::endl;
	std::cout << "S.inverse() = " << std::endl << S.inverse() << std::endl;
	std::cout << "P*jH.transpose()*S.inverse() = K = " << std::endl << K << std::endl;
	std::cout << "X + K*Y = X = " << std::endl << X << std::endl;
	std::cout << "(I - K*jH)*P = P = " << std::endl << P << std::endl;
}

bool judge_moving(void)
{
	const float threshold_w = 1.0;
	const float threshold_a = 1.0;
	// std::cout << "record[record.size()-1].wx - ave.wx = " << record[record.size()-1].wx - ave.wx << std::endl;
	if(fabs(record[record.size()-1].wx - ave.wx)>threshold_w)	return true;
	if(fabs(record[record.size()-1].wy - ave.wy)>threshold_w)	return true;
	if(fabs(record[record.size()-1].wz - ave.wz)>threshold_w)	return true;
	if(fabs(record[record.size()-1].ax - ave.ax)>threshold_a)	return true;
	if(fabs(record[record.size()-1].ay - ave.ay)>threshold_a)	return true;
	if(fabs(record[record.size()-1].az - ave.az)>threshold_a)	return true;
	// std::cout << "non move" << std::endl;
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
	if(record.size()>num_record)	record.erase(record.begin());
	
	ave = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	for(size_t i=0;i<record.size();i++){
		ave.wx += record[i].wx/(float)record.size();
		ave.wy += record[i].wy/(float)record.size();
		ave.wz += record[i].wz/(float)record.size();
		ave.ax += record[i].ax/(float)record.size();
		ave.ay += record[i].ay/(float)record.size();
		ave.az += record[i].az/(float)record.size();
	}
	// std::cout << "ave.wx = " << ave.wx << std::endl;
	// std::cout << "ave.wy = " << ave.wy << std::endl;
	// std::cout << "ave.wz = " << ave.wz << std::endl;
	// std::cout << "ave.ax = " << ave.ax << std::endl;
	// std::cout << "ave.ay = " << ave.ay << std::endl;
	// std::cout << "ave.az = " << ave.az << std::endl;
	
	if(record.size()>num_record-1)	moving = judge_moving();
	if(moving){
		std::cout << "moved" << std::endl;
		tf::Quaternion tmp_q = tf::createQuaternionFromRPY(X(0, 0), X(1, 0), X(2, 0));
		quaternionTFToMsg(tmp_q, ini_pose);
	}

	if(record.size()==1){
		double roll, pitch, yaw;
		tf::Quaternion tmp_q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
		tf::Matrix3x3(tmp_q).getRPY(roll, pitch, yaw);
		X <<	roll,
		  		pitch,
				yaw;
	}

	if(record.size()>num_record-1&&!moving){
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
		jF <<	1.0+U(1, 0)*tan(X(1, 0))*cos(X(0, 0)) - U(2, 0)*tan(X(1, 0))*sin(X(0, 0)),
					U(1, 0)*sin(X(0, 0))/( cos(X(1, 0))*cos(X(1, 0)) ) - U(2, 0)*cos(X(0, 0))/( cos(X(1, 0))*cos(X(1, 0)) ),
						0,
				-U(1, 0)*sin(X(0, 0)) - U(2, 0)*cos(X(0, 0)),
					1.0,
						0,
				U(1, 0)/cos(X(1, 0))*cos(X(0, 0)) - U(2, 0)/cos(X(1, 0))*sin(X(0, 0)),
					U(1, 0)*sin(X(0, 0 ))*sin(X(1, 0))/( cos(X(1, 0))*cos(X(1, 0)) ) + U(2, 0)*cos(X(0, 0))*sin(X(1, 0))/( cos(X(1, 0))*cos(X(1, 0)) ),
						1.0;
		
		Eigen::MatrixXf Q(3, 3);
		
		X = A*X + B*U;
		P = jF*P*jF.transpose();
		
		std::cout << "~ prediction ~" << std::endl;
		// std::cout << "A = " << std::endl << A << std::endl;
		std::cout << "U = " << std::endl << U << std::endl;
		std::cout << "B = " << std::endl << B << std::endl;
		std::cout << "A*X + B*U = X = " << std::endl << X << std::endl;
		std::cout << "jF = " << std::endl << jF << std::endl;
		std::cout << "jF*P*jF.transpose() = P = " << std::endl << P << std::endl;
		
		
		strapdown1();
	}
	
	// first_callback = false;
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_alignment");
	ros::NodeHandle nh;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Subscriber sub_imu = nh.subscribe("/imu/data", 10, callback_imu);
	ros::Publisher pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/ini_pose",1);

	X <<	0.0,
	  		0.0,
			0.0;
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
