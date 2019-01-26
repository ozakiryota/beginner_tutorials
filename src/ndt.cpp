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
		/*odom*/
		nav_msgs::Odometry odom_now;
		nav_msgs::Odometry odom_last;
		/*flags*/
		bool first_callback_odom = true;
		bool is_transforemed = false;
	public:
		NDT();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Transformation(void);
		void Visualization(void);
		void Publication(void);
};

NDT::NDT()
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &NDT::CallbackPC, this);
	sub_odom = nh.subscribe("/tinypower/odom/republished", 1, &NDT::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/ndt_odom", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");
}

void NDT::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// std::cout << "CALLBACK PC" << std::endl;
	pcl::fromROSMsg(*msg, *cloud_now);
	
	if(cloud_last->points.empty())	*cloud_last = *cloud_now;
	if(is_transforemed)	*cloud_last = *cloud_now;

	is_transforemed = false;
}

void NDT::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// std::cout << "CALLBACK ODOM" << std::endl;
	odom_now = *msg;

	if(!first_callback_odom && !is_transforemed){
		Transformation();
		is_transforemed = true;
		Visualization();
		Publication();
	}

	odom_last = odom_now;
	first_callback_odom = false;
}

void NDT::Transformation(void)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud(cloud_now);
	approximate_voxel_filter.filter(*filtered_cloud);

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize (0.1);
	ndt.setResolution (1.0);
	ndt.setMaximumIterations(35);
	// ndt.setInputSource(cloud_now);
	ndt.setInputSource(filtered_cloud);
	ndt.setInputTarget(cloud_last);

	Eigen::Quaternionf q_pose_now(
		odom_now.pose.pose.orientation.w,
		odom_now.pose.pose.orientation.x,
		odom_now.pose.pose.orientation.y,
		odom_now.pose.pose.orientation.z);
	Eigen::Quaternionf q_pose_last(
		odom_last.pose.pose.orientation.w,
		odom_last.pose.pose.orientation.x,
		odom_last.pose.pose.orientation.y,
		odom_last.pose.pose.orientation.z);
	Eigen::Quaternionf relative_rot = q_pose_now*q_pose_last.inverse();
	Eigen::AngleAxisf init_rotation(relative_rot);
	
	Eigen::Quaternionf odom_translation(
		0.0,
		odom_now.pose.pose.position.x - odom_last.pose.pose.position.x,
		odom_now.pose.pose.position.y - odom_last.pose.pose.position.y,
		odom_now.pose.pose.position.z - odom_last.pose.pose.position.z);
	odom_translation = relative_rot.inverse()*odom_translation*relative_rot;
	Eigen::Translation3f init_translation(odom_translation.x(), odom_translation.y(), odom_translation.z());
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();
	ndt.align(*cloud_now, init_guess);

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged () 
		<< std::endl << " score: " << ndt.getFitnessScore () << std::endl;
}

void NDT::Visualization(void)
{
	viewer.removeAllPointClouds();

	viewer.addPointCloud<pcl::PointXYZ>(cloud_now, "cloud_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud_now");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_now");

	viewer.addPointCloud<pcl::PointXYZ>(cloud_last, "cloud_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_last");
	
	viewer.spinOnce();
}

void NDT::Publication(void)
{
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ndt");
	
	NDT ndt;

	ros::spin();
}
