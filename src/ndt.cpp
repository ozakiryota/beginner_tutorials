#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/filters/passthrough.h>

class NDT{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_pc;
		ros::Subscriber sub_odom;
		/*publish*/
		ros::Publisher pub_odom;
		/*viewer*/
		pcl::visualization::PCLVisualizer viewer{"ndt"};
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_now {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_last {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_input {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr init_guess_point {new pcl::PointCloud<pcl::PointNormal>};
		/*odom*/
		nav_msgs::Odometry odom_ndt;
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*time*/
		// ros::Time time_now_odom;
		// ros::Time time_last_odom;
		/*flags*/
		bool first_callback_odom = true;
		// bool is_transforemed = false;
	public:
		NDT();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Compute(void);
		void Transformation(void);
		Eigen::Quaternionf QuatMsgToEigen(geometry_msgs::Quaternion q_msg);
		geometry_msgs::Quaternion QuatEigenToMsg(Eigen::Quaternionf q_eigen);
		void Visualization(void);
		void Publication(void);
};

NDT::NDT()
{
	// sub_pc = nh.subscribe("/velodyne_points", 1, &NDT::CallbackPC, this);
	sub_pc = nh.subscribe("/rm_ground2", 1, &NDT::CallbackPC, this);
	sub_odom = nh.subscribe("/tinypower/odom/republished", 1, &NDT::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_ndt", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);
	InitializeOdom(odom_ndt);
	InitializeOdom(odom_now);
	InitializeOdom(odom_last);
	init_guess_point->points.resize(1);
}

void NDT::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/odom_ndt";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void NDT::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK PC" << std::endl;
	
	if(cloud_last->points.empty()){
		pcl::fromROSMsg(*msg, *cloud_now);
		*cloud_last = *cloud_now;
	}
	else{
		*cloud_last = *cloud_now;
		pcl::fromROSMsg(*msg, *cloud_now);
	}

	const double range = 20;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_now);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-range, range);
	pass.filter(*cloud_now);
	pass.setInputCloud(cloud_now);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-range, range);
	pass.filter(*cloud_now);

	// is_transforemed = false;
}

void NDT::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "CALLBACK ODOM" << std::endl;

	// double dt = (time_now_odom - time_last_odom).toSec();
	if(first_callback_odom){
		odom_now = *msg;
		odom_last = odom_now;
		// time_now_odom = ros::Time::now();
		// time_last_odom = time_now_odom;
	}
	else{
		odom_last = odom_now;
		odom_now = *msg;
		// time_last_odom = time_now_odom;
		// time_now_odom = ros::Time::now();
	}

	// if(!first_callback_odom && !is_transforemed){
	// 	Transformation(dt);
	// 	is_transforemed = true;
	// 	Visualization();
	// 	Publication();
	// }

	first_callback_odom = false;
}

void NDT::Compute(void)
{
	// double dt = (time_now_odom - time_last_odom).toSec();
	Transformation();
	Visualization();
	Publication();
}

void NDT::Transformation(void)
{
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.5, 0.5, 0.5);
	approximate_voxel_filter.setInputCloud(cloud_now);
	approximate_voxel_filter.filter(*filtered_input);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target (new pcl::PointCloud<pcl::PointXYZ>);
	approximate_voxel_filter.setInputCloud(cloud_last);
	approximate_voxel_filter.filter(*filtered_target);

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	// ndt.setResolution(1.0);
	ndt.setResolution(3.0);
	ndt.setMaximumIterations(35);
	// ndt.setInputSource(filtered_input);
	// ndt.setInputTarget(filtered_target);
	ndt.setInputSource(filtered_target);
	ndt.setInputTarget(filtered_input);

	Eigen::Quaternionf q_pose_now = QuatMsgToEigen(odom_now.pose.pose.orientation);
	Eigen::Quaternionf q_pose_last = QuatMsgToEigen(odom_last.pose.pose.orientation);
	// Eigen::Quaternionf q_relative_rotation = q_pose_now*q_pose_last.inverse();
	Eigen::Quaternionf q_relative_rotation = q_pose_last.inverse()*q_pose_now;
	q_relative_rotation.normalize();
	Eigen::Quaternionf q_global_move(
		0.0,
		odom_now.pose.pose.position.x - odom_last.pose.pose.position.x,
		odom_now.pose.pose.position.y - odom_last.pose.pose.position.y,
		odom_now.pose.pose.position.z - odom_last.pose.pose.position.z);
	Eigen::Quaternionf q_local_move = q_pose_last.inverse()*q_global_move*q_pose_last;
	Eigen::Translation3f init_translation(q_local_move.x(), q_local_move.y(), q_local_move.z());
	Eigen::AngleAxisf init_rotation(q_relative_rotation);
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();
	ndt.align(*filtered_input, init_guess);
	// ndt.align(*filtered_input);

	std::cout << "init_guess" << std::endl << init_guess << std::endl;
	std::cout << "init_rotation" << std::endl;
	std::cout << init_rotation.angle() << std::endl;
	std::cout << init_rotation.axis() << std::endl;
	init_guess_point->points[0].x = init_guess(0, 3);
	init_guess_point->points[0].y = init_guess(1, 3);
	init_guess_point->points[0].z = init_guess(2, 3);
	Eigen::Quaternionf q_orient(0, 1, 0, 0);
	q_orient = q_relative_rotation*q_orient*q_relative_rotation.inverse();
	init_guess_point->points[0].normal_x = q_orient.x();
	init_guess_point->points[0].normal_y = q_orient.y();
	init_guess_point->points[0].normal_z = q_orient.z();

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged () 
		<< std::endl << " score: " << ndt.getFitnessScore () << std::endl;
	std::cout << "ndt.getFinalTransformation()" << std::endl << ndt.getFinalTransformation() << std::endl;

	Eigen::Matrix4f m_transformation = ndt.getFinalTransformation();

	Eigen::Matrix3f m_rot = m_transformation.block(0, 0, 3, 3);
	Eigen::Quaternionf q_rot(m_rot);
	q_rot.normalize();
	Eigen::Quaternionf q_pose = QuatMsgToEigen(odom_ndt.pose.pose.orientation);
	q_pose = q_pose*q_rot;
	q_pose.normalize();
	odom_ndt.pose.pose.orientation = QuatEigenToMsg(q_pose);

	Eigen::Quaternionf q_trans(
		0.0,
		m_transformation(0, 3),
		m_transformation(1, 3),
		m_transformation(2, 3));
	q_trans = q_pose*q_trans*q_pose.inverse();
	std::cout << q_trans.x() << std::endl;
	std::cout << q_trans.y() << std::endl;
	std::cout << q_trans.z() << std::endl;
	std::cout << q_trans.w() << std::endl;
	odom_ndt.pose.pose.position.x += q_trans.x();
	odom_ndt.pose.pose.position.y += q_trans.y();
	odom_ndt.pose.pose.position.z += q_trans.z();
}

Eigen::Quaternionf NDT::QuatMsgToEigen(geometry_msgs::Quaternion q_msg){
	Eigen::Quaternionf q_eigen(
		(float)q_msg.w,
		(float)q_msg.x,
		(float)q_msg.y,
		(float)q_msg.z);
	q_eigen.normalize();
	return q_eigen;
}

geometry_msgs::Quaternion NDT::QuatEigenToMsg(Eigen::Quaternionf q_eigen)
{
	geometry_msgs::Quaternion q_msg;
	q_msg.x = (double)q_eigen.x();
	q_msg.y = (double)q_eigen.y();
	q_msg.z = (double)q_eigen.z();
	q_msg.w = (double)q_eigen.w();
	return q_msg;
}

void NDT::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud<pcl::PointXYZ>(cloud_now, "cloud_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud_now");

	viewer.addPointCloud<pcl::PointXYZ>(cloud_last, "cloud_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud_last");

	viewer.addPointCloud<pcl::PointXYZ>(filtered_input, "filtered_input");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "filtered_input");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "filtered_input");

	// viewer.addPointCloud<pcl::PointXYZ>(init_guess_point, "init_guess_point");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "init_guess_point");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, "init_guess_point");
	viewer.addPointCloudNormals<pcl::PointNormal>(init_guess_point, 1, 1.0, "init_guess_point");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "init_guess_point");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "init_guess_point");
	
	viewer.spinOnce();
}

void NDT::Publication(void)
{
	odom_ndt.header.stamp = ros::Time::now();
	pub_odom.publish(odom_ndt);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ndt");
	
	NDT ndt;

	// ros::spin();

	ros::Rate loop_rate(10);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
		ndt.Compute();
	}
}
