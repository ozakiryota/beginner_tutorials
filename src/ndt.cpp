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
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr init_guess_point {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr final_point {new pcl::PointCloud<pcl::PointNormal>};
		/*odom*/
		nav_msgs::Odometry odom_ndt;
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*flags*/
		bool first_callback_odom = true;
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
	sub_pc = nh.subscribe("/velodyne_points", 1, &NDT::CallbackPC, this);
	// sub_pc = nh.subscribe("/rm_ground2", 1, &NDT::CallbackPC, this);
	// sub_odom = nh.subscribe("/tinypower/odom/republished", 1, &NDT::CallbackOdom, this);
	sub_odom = nh.subscribe("/gyrodometry", 1, &NDT::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_ndt", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
	viewer.setCameraPosition(0.0, 0.0, 80.0, 0.0, 0.0, 0.0);
	InitializeOdom(odom_ndt);
	InitializeOdom(odom_now);
	InitializeOdom(odom_last);
	init_guess_point->points.resize(1);
	final_point->points.resize(1);
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

	// const double range = 45;
	// pcl::PassThrough<pcl::PointXYZ> pass;
	// pass.setInputCloud(cloud_now);
	// pass.setFilterFieldName("x");
	// pass.setFilterLimits(-range, range);
	// pass.filter(*cloud_now);
	// pass.setInputCloud(cloud_now);
	// pass.setFilterFieldName("y");
	// pass.setFilterLimits(-range, range);
	// pass.filter(*cloud_now);
}

void NDT::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "CALLBACK ODOM" << std::endl;

	if(first_callback_odom){
		odom_now = *msg;
		odom_last = odom_now;
	}
	else{
		odom_last = odom_now;
		odom_now = *msg;
	}

	first_callback_odom = false;
}

void NDT::Compute(void)
{
	std::cout << "COMPUTE" << std::endl;
	Transformation();
	Visualization();
	Publication();
}

void NDT::Transformation(void)
{
	/*down sampling*/
	const double leafsize = 0.1;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter_input;
	approximate_voxel_filter_input.setLeafSize(leafsize, leafsize, leafsize);
	approximate_voxel_filter_input.setInputCloud(cloud_now);
	approximate_voxel_filter_input.filter(*filtered_input);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter_target;
	approximate_voxel_filter_target.setLeafSize(leafsize, leafsize, leafsize);
	approximate_voxel_filter_target.setInputCloud(cloud_last);
	approximate_voxel_filter_target.filter(*filtered_target);
	
	/*set parameters*/
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	// ndt.setTransformationEpsilon(0.01);
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	// ndt.setResolution(1.0);
	ndt.setResolution(0.5);
	ndt.setMaximumIterations(35);
	// ndt.setInputSource(filtered_input);
	// ndt.setInputTarget(filtered_target);
	ndt.setInputSource(filtered_target);
	ndt.setInputTarget(filtered_input);

	/*initial guess*/
	Eigen::Quaternionf q_pose_now = QuatMsgToEigen(odom_now.pose.pose.orientation);
	Eigen::Quaternionf q_pose_last = QuatMsgToEigen(odom_last.pose.pose.orientation);
	Eigen::Quaternionf q_relative_rotation = q_pose_now.inverse()*q_pose_last;
	q_relative_rotation.normalize();
	Eigen::Quaternionf q_global_move(
		0.0,
		odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		odom_last.pose.pose.position.z - odom_now.pose.pose.position.z);
	Eigen::Quaternionf q_local_move = q_pose_now.inverse()*q_global_move*q_pose_now;
	Eigen::Translation3f init_translation(q_local_move.x(), q_local_move.y(), q_local_move.z());
	Eigen::AngleAxisf init_rotation(q_relative_rotation);
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();

	/*align*/
	ndt.align(*filtered_input, init_guess);
	// ndt.align(*filtered_input);

	/*print*/
	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged () 
		<< std::endl << " score: " << ndt.getFitnessScore () << std::endl;
	std::cout << "ndt.getFinalTransformation()" << std::endl << ndt.getFinalTransformation() << std::endl;
	std::cout << "init_guess" << std::endl << init_guess << std::endl;

	/*convert to /odom*/
	Eigen::Matrix4f m_transformation = ndt.getFinalTransformation();
	Eigen::Matrix3f m_rot = m_transformation.block(0, 0, 3, 3);
	Eigen::Quaternionf q_rot(m_rot);
	q_rot.normalize();
	q_rot = q_rot.inverse();
	Eigen::Quaternionf q_pose = QuatMsgToEigen(odom_ndt.pose.pose.orientation);
	q_pose = q_pose*q_rot;
	q_pose.normalize();
	// odom_ndt.pose.pose.orientation = QuatEigenToMsg((q_pose*q_rot).normalized());
	odom_ndt.pose.pose.orientation = QuatEigenToMsg(q_pose);

	// double roll, pitch, yaw;
	// tf::Matrix3x3 tmp(
	// 	m_rot(0,0),	m_rot(0,1), m_rot(0,2),
	// 	m_rot(1,0),	m_rot(1,1),	m_rot(1,2),
	// 	m_rot(2,0),	m_rot(2,1),	m_rot(2,2)
	// );
	// tmp.getRPY(roll, pitch, yaw);
	// std::cout << "RPY: " << roll/M_PI*180 << ", " << pitch/M_PI*180 << ", " << yaw/M_PI*180 << std::endl;
	// geometry_msgs::Quaternion q_ = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

	Eigen::Quaternionf q_trans(
		0.0,
		-m_transformation(0, 3),
		-m_transformation(1, 3),
		-m_transformation(2, 3)
	);
	q_trans = q_pose*q_trans*q_pose.inverse();
	std::cout << q_trans.x() << std::endl;
	std::cout << q_trans.y() << std::endl;
	std::cout << q_trans.z() << std::endl;
	std::cout << q_trans.w() << std::endl;
	odom_ndt.pose.pose.position.x += q_trans.x();
	odom_ndt.pose.pose.position.y += q_trans.y();
	odom_ndt.pose.pose.position.z += q_trans.z();
}

Eigen::Quaternionf NDT::QuatMsgToEigen(geometry_msgs::Quaternion q_msg)
{
	Eigen::Quaternionf q_eigen(
		(float)q_msg.w,
		(float)q_msg.x,
		(float)q_msg.y,
		(float)q_msg.z
	);
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
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud_now");

	viewer.addPointCloud<pcl::PointXYZ>(cloud_last, "cloud_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "cloud_last");

	// viewer.addPointCloud<pcl::PointXYZ>(filtered_input, "filtered_input");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "filtered_input");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "filtered_input");

	viewer.addPointCloud<pcl::PointXYZ>(filtered_target, "filtered_target");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "filtered_target");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "filtered_target");

	// viewer.addPointCloudNormals<pcl::PointNormal>(init_guess_point, 1, 1.0, "init_guess_point");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "init_guess_point");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "init_guess_point");
    //
	// viewer.addPointCloudNormals<pcl::PointNormal>(final_point, 1, 1.0, "final_point");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "final_point");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "final_point");
	
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
		std::cout << "----------" << std::endl;
		ros::spinOnce();
		ndt.Compute();
		loop_rate.sleep();
	}
}
