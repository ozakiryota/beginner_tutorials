/*
 * doubleekf.cpp
 *
 * X = |Φ Θ Ψ|^T
*/

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

/*global variables*/
tf::Quaternion q_pose(0.0, 0.0, 0.0, 1.0);
bool inipose_is_available = false;
bool bias_is_available = false;
sensor_msgs::Imu imu;
sensor_msgs::Imu bias;

const int num_state = 3;
Eigen::MatrixXd Xsub(num_state, 1);
Eigen::MatrixXd Psub(num_state, num_state);
Eigen::MatrixXd Xmain(num_state, 1);
Eigen::MatrixXd Pmain(num_state, num_state);
ros::Time time_now;
ros::Time time_last;

void input_pose(geometry_msgs::Pose& pose, Eigen::MatrixXd Mat)
{
	tf::Quaternion q_ = tf::createQuaternionFromRPY(Mat(0, 0), Mat(1, 0), Mat(2, 0));
	quaternionTFToMsg(q_, pose.orientation);

	// q_pose.normalize();
	// quaternionTFToMsg(q_pose, pose.orientation);

	// pose.position.x = 0.0;
	// pose.position.y = 0.0;
	// pose.position.z = 0.0;
}

int count_usingwalls = 0;
void callback_usingwalls(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	const int num_obs = 3;
	
	if(inipose_is_available){
		std::cout << count_usingwalls << ": ";
		count_usingwalls++;
		std::cout << "CALLBACK OBSERVATION USINGWALLS" << std::endl;
		
		pcl::PointCloud<pcl::PointNormal>::Ptr g_vector (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(*msg, *g_vector);

		const double g = -9.80665;
		double gx = g_vector->points[0].normal_x*g; 
		double gy = g_vector->points[0].normal_y*g; 
		double gz = g_vector->points[0].normal_z*g; 
		// std::cout << "g_vector->points[0] = " << g_vector->points[0] << std::endl;

		Eigen::MatrixXd Z(num_obs, 1);
		Z <<	atan2(gy, gz),
		  		atan2(-gx, sqrt(gy*gy + gz*gz)),
				Xsub(2, 0);

		Eigen::MatrixXd H(num_obs, num_state);
		H <<	1,	0,	0,
				0,	1,	0,
				0, 	0,	1;

		Eigen::MatrixXd jH(num_obs, num_state);
		jH <<	1,	0,	0,
				0,	1,	0,
				0,	0,	1;

		Eigen::MatrixXd R(num_obs, num_obs);
		const double sigma = 1.0e-1;
		R = sigma*Eigen::MatrixXd::Identity(num_obs, num_obs);

		Eigen::MatrixXd Y(num_obs, 1);
		Eigen::MatrixXd S(num_obs, num_obs);
		Eigen::MatrixXd K(num_state, num_obs);
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_state, num_state);

		Y = Z - H*Xsub;
		for(int i=0;i<3;i++){
			if(Y(i, 0)>M_PI)	Y(i, 0) -= 2.0*M_PI;
			if(Y(i, 0)<-M_PI)	Y(i, 0) += 2.0*M_PI;
		}
		S = jH*Psub*jH.transpose() + R;
		K = Psub*jH.transpose()*S.inverse();
		K(2, 0) = 0.0;	//temporary repair
		K(2, 1) = 0.0;	//temporary repair
		K(2, 2) = 0.0;	//temporary repair
		Xsub = Xsub + K*Y;
		Psub = (I - K*jH)*Psub;

		std::cout << "Y = " << std::endl << Y << std::endl;
		std::cout << "K*Y = " << std::endl << K*Y << std::endl;
	}
}

tf::Quaternion q_slam_now;
tf::Quaternion q_slam_last;
void prediction_slam(void)
{
	tf::Quaternion q_relative_rotation = q_slam_now*q_slam_last.inverse();
	q_relative_rotation = q_slam_last.inverse()*q_slam_now;
	q_relative_rotation.normalize();

	double roll = Xsub(0, 0);
	double pitch = Xsub(1, 0);
	double yaw = Xsub(2, 0);

	double delta_r, delta_p, delta_y;
	tf::Matrix3x3(q_relative_rotation).getRPY(delta_r, delta_p, delta_y);
	// Eigen::MatrixXd U(3, 1);
	// U <<	delta_r,
	//   		delta_p,
	// 		delta_y;
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(num_state, num_state);
	Eigen::MatrixXd B = Eigen::MatrixXd::Identity(num_state, num_state);
	// Eigen::MatrixXd F = A*Xsub + B*U;
	Eigen::MatrixXd F(num_state, 1);
	F <<	roll + (delta_r + sin(roll)*tan(pitch)*delta_p + cos(roll)*tan(pitch)*delta_y),
			pitch + (cos(roll)*delta_p - sin(roll)*delta_y),
			yaw + (sin(roll)/cos(pitch)*delta_p + cos(roll)/cos(pitch)*delta_y);
	for(int i=0;i<num_state;i++){
		if(F(i, 0)>M_PI)    F(i, 0) -= 2.0*M_PI;
		if(F(i, 0)<-M_PI)   F(i, 0) += 2.0*M_PI;
	}
	double dfdx[num_state][num_state];
	dfdx[0][0] = 1.0 + (cos(roll)*tan(pitch)*delta_p - sin(roll)*tan(pitch)*delta_y);
	dfdx[0][1] = (sin(roll)/cos(pitch)/cos(pitch)*delta_p + cos(roll)/cos(pitch)/cos(pitch)*delta_y);
	dfdx[0][2] = 0.0;
	dfdx[1][0] = (-sin(roll)*delta_p - cos(roll)*delta_y);
	dfdx[1][1] = 1.0;
	dfdx[1][2] = 0.0;
	dfdx[2][0] = (cos(roll)/cos(pitch)*delta_p - sin(roll)/cos(pitch)*delta_y);
	dfdx[2][1] = (-sin(roll)/sin(pitch)*delta_p - cos(roll)/sin(pitch)*delta_y);
	dfdx[2][2] = 1.0;
	Eigen::MatrixXd jF(num_state, num_state);
	for(int i=0;i<num_state;i++){
		for(int j=0;j<num_state;j++)    jF(i, j) = dfdx[i][j];
	}
	const double sigma = 1.0e-1;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(num_state, num_state);

	tf::Quaternion q_pose_tmp = q_pose;
	q_pose = q_relative_rotation*q_pose_tmp;
	// q_pose = q_pose_tmp*q_relative_rotation;
	// q_pose = q_pose_tmp*q_relative_rotation*q_pose_tmp;

	// // q_pose = (q_slam_now*q_slam_last_inv)*q_pose_tmp;
	// // q_pose = q_pose_tmp*(q_slam_now*q_slam_last_inv);
	// // q_pose = (q_slam_now*q_slam_last.inverse())*q_slam_last;
	//
	q_pose.normalize();
	// tf::Matrix3x3(q_pose).getRPY(Xsub(0, 0), Xsub(1, 0), Xsub(2, 0));

	Xsub = F;
	Psub = jF*Psub*jF.transpose() + Q;
	
	q_slam_last = q_slam_now;
	
	if(fabs(Xsub(0, 0))>M_PI) std::cout << "fabs(Xsub(0, 0))>M_PI" << std::endl;
	if(fabs(Xsub(1, 0))>M_PI) std::cout << "fabs(Xsub(1, 0))>M_PI" << std::endl;
	if(fabs(Xsub(2, 0))>M_PI) std::cout << "fabs(Xsub(2, 0))>M_PI" << std::endl;
}

bool firstcallback = true;
void callback_slam(const geometry_msgs::PoseStampedConstPtr& msg)
{
	// std::cout << "CALLBACK SLAM" << std::endl;
	
	q_slam_now = tf::Quaternion(msg->pose.orientation.z, -msg->pose.orientation.x, -msg->pose.orientation.y, msg->pose.orientation.w);
	q_slam_now.normalize();
	
	if(inipose_is_available)	prediction_slam();
	else	q_slam_last = q_slam_now;
	
	// if(firstcallback){
	// 	q_slam_last = q_slam_now;
	// 	tf::Matrix3x3(q_pose).getRPY(Xsub(0, 0), Xsub(1, 0), Xsub(2, 0));
	// 	if(fabs(Xsub(0, 0))>M_PI) std::cout << "fabs(Xsub(0, 0))>M_PI" << std::endl;
	// 	if(fabs(Xsub(1, 0))>M_PI) std::cout << "fabs(Xsub(1, 0))>M_PI" << std::endl;
	// 	if(fabs(Xsub(2, 0))>M_PI) std::cout << "fabs(Xsub(2, 0))>M_PI" << std::endl;
	// }
	// else	prediction_slam();
	// firstcallback = false;
}

void prediction_imu(double dt)
{
	double roll = Xmain(0, 0);
	double pitch = Xmain(1, 0);
	double yaw = Xmain(2, 0);

	double delta_r = imu.angular_velocity.x*dt;
	double delta_p = imu.angular_velocity.y*dt;
	double delta_y = imu.angular_velocity.z*dt;
	if(bias_is_available){
		delta_r -= bias.angular_velocity.x*dt;
		delta_p -= bias.angular_velocity.y*dt;
		delta_y -= bias.angular_velocity.z*dt;
	}

	Eigen::MatrixXd F(num_state, 1);
	F <<	roll + (delta_r + sin(roll)*tan(pitch)*delta_p + cos(roll)*tan(pitch)*delta_y),
			pitch + (cos(roll)*delta_p - sin(roll)*delta_y),
			yaw + (sin(roll)/cos(pitch)*delta_p + cos(roll)/cos(pitch)*delta_y);
	double dfdx[num_state][num_state];
	dfdx[0][0] = 1.0 + (cos(roll)*tan(pitch)*delta_p - sin(roll)*tan(pitch)*delta_y);
	dfdx[0][1] = (sin(roll)/cos(pitch)/cos(pitch)*delta_p + cos(roll)/cos(pitch)/cos(pitch)*delta_y);
	dfdx[0][2] = 0.0;
	dfdx[1][0] = (-sin(roll)*delta_p - cos(roll)*delta_y);
	dfdx[1][1] = 1.0;
	dfdx[1][2] = 0.0;
	dfdx[2][0] = (cos(roll)/cos(pitch)*delta_p - sin(roll)/cos(pitch)*delta_y);
	dfdx[2][1] = (-sin(roll)/sin(pitch)*delta_p - cos(roll)/sin(pitch)*delta_y);
	dfdx[2][2] = 1.0;
	Eigen::MatrixXd jF(num_state, num_state);
	for(int i=0;i<num_state;i++){
		for(int j=0;j<num_state;j++)    jF(i, j) = dfdx[i][j];
	}
	const double sigma = 1.0e-1;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(num_state, num_state);
	
	Xmain = F;
	Pmain = jF*Pmain*jF.transpose() + Q;
}

void callback_imu(const sensor_msgs::ImuConstPtr& msg)
{
	// std::cout << "CALLBACK IMU" << std::endl;
	imu = *msg;
	time_now = ros::Time::now();
	double dt = (time_now - time_last).toSec();
	time_last = time_now;
	
	if(inipose_is_available)	prediction_imu(dt);
}

void callback_bias(const sensor_msgs::ImuConstPtr& msg)
{ 
	// std::cout << "CALLBACK BIAS" << std::endl;
	bias = *msg;
	bias_is_available = true;
}

void callback_inipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		quaternionMsgToTF(*msg, q_pose);
		// q_pose = tf::Quaternion(0.0, 0.5, 0.0, 1.0);	//for test
		q_pose.normalize();
		tf::Matrix3x3(q_pose).getRPY(Xsub(0, 0), Xsub(1, 0), Xsub(2, 0));
		tf::Matrix3x3(q_pose).getRPY(Xmain(0, 0), Xmain(1, 0), Xmain(2, 0));
		inipose_is_available = true;
		std::cout << "inipose_is_available = " << inipose_is_available << std::endl;
		std::cout << "initial pose = " << std::endl << Xsub << std::endl;
	}
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "doubleekf");
	ros::NodeHandle nh;

	/*time*/
	time_now = ros::Time::now();
	time_last = ros::Time::now();

	/*sub*/
	ros::Subscriber sub_inipose = nh.subscribe("/initial_pose", 1, callback_inipose);
	// ros::Subscriber sub_bias = nh.subscribe("/imu_bias", 1, callback_bias);
	// ros::Subscriber sub_imu = nh.subscribe("/imu/data", 1, callback_imu);
	ros::Subscriber sub_obs1 = nh.subscribe("/g_usingwalls", 1, callback_usingwalls);
	ros::Subscriber sub_obs2 = nh.subscribe("/lsd_slam/pose", 1, callback_slam);
	
	/*pub*/
	ros::Publisher pub_pose = nh.advertise<geometry_msgs::Pose>("/pose_doubleekf", 1);

	/*variables*/
	geometry_msgs::Pose pose;

	/*initialization*/
	Xsub = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	Psub = 100.0*Eigen::MatrixXd::Identity(num_state, num_state);
	Xmain = Eigen::MatrixXd::Constant(num_state, 1, 0.0);
	Pmain = 100.0*Eigen::MatrixXd::Identity(num_state, num_state);
	q_pose.normalize();

	/*loop*/
	ros::Rate loop_rate(200);
	while(ros::ok()){
		// std::cout << "loop" << std::endl;
		ros::spinOnce();

		input_pose(pose, Xsub);
		// input_pose(pose, Xmain);
		pub_pose.publish(pose);
        //
		loop_rate.sleep();
	}
}
