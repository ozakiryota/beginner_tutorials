/*
 *	pc_fitting_walls.cpp
 */

#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Pose.h>
// #include <Eigen/Core>
#include <tf/tf.h>
#include <pcl/common/transforms.h>

Eigen::MatrixXd frame_rotation(geometry_msgs::Quaternion q, Eigen::MatrixXd X, bool from_global_to_local)
{
	if(!from_global_to_local)    q.w *= -1;
	Eigen::MatrixXd Rot(3, 3); 
	Rot <<	q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z,  2*(q.x*q.y + q.w*q.z),	2*(q.x*q.z - q.w*q.y),
		2*(q.x*q.y - q.w*q.z),	q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z,	2*(q.y*q.z + q.w*q.x),
		2*(q.x*q.z + q.w*q.y),	2*(q.y*q.z - q.w*q.x),	q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	// std::cout << "Rot*X = " << std::endl << Rot*X << std::endl;
	if(from_global_to_local)    return Rot*X;
	else    return Rot.inverse()*X;
}

class PointCloudFittingWalls{
	private:
		/*node hundle*/
		ros::NodeHandle nh;
		/*sub*/
		ros::Subscriber sub_pose;
		ros::Subscriber sub_cloud;
		/*pub*/
		ros::Publisher pub_cloud;
		ros::Publisher pub_normals;
		ros::Publisher pub_g;
		/*pcl clouds*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_flipped {new pcl::PointCloud<pcl::PointXYZINormal>};	//just for viewing
		pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr main_normals {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr g_vector {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_before_clustering {new pcl::PointCloud<pcl::PointXYZINormal>};
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_after_clustering {new pcl::PointCloud<pcl::PointXYZINormal>};	
		pcl::PointCloud<pcl::PointNormal>::Ptr g_last {new pcl::PointCloud<pcl::PointNormal>};

		/*variables*/
		// sensor_msgs::PointCloud2 cloud_ros;
		// sensor_msgs::PointCloud2 normals_ros;
		pcl::visualization::PCLVisualizer viewer;
		std::vector<double> g_local;
		/*const variables*/
		std::string pc_frame_id = "/odom3d_with_posemsg";
		/*flags*/
		bool g_local_is_available;
	public:
		PointCloudFittingWalls();
		void callback_pose(const geometry_msgs::PoseConstPtr& msg);
		void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg);
		void pcl_visualization(void);
};

PointCloudFittingWalls::PointCloudFittingWalls()
{
	sub_pose = nh.subscribe("/pose_dualekf", 1, &PointCloudFittingWalls::callback_pose, this);
	sub_cloud = nh.subscribe("/velodyne_points", 1, &PointCloudFittingWalls::callback_cloud, this);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud",1);
	pub_normals = nh.advertise<sensor_msgs::PointCloud2>("/normals",1);
	pub_g = nh.advertise<sensor_msgs::PointCloud2>("/g_usingwalls",1);

	viewer = pcl::visualization::PCLVisualizer("pc_fitting_walls");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");

	g_local_is_available = false;
}

void PointCloudFittingWalls::callback_pose(const geometry_msgs::PoseConstPtr& msg)
{
	Eigen::MatrixXd G_global(3, 1);
	G_global <<	0.0,
			 	0.0,
				-9.80665;
	Eigen::MatrixXd G_local(3, 1);
	G_local = frame_rotation(msg->orientation, G_global, true);
	g_local = {G_local(0, 0), G_local(1, 0), G_local(2, 0)};

	std::cout << "g_local = " << g_local[0] << ", " << g_local[1] << ", " << g_local[2] << ", "  << std::endl;

	g_local_is_available = true;

	pcl::PointNormal tmp;
	tmp.x = 0.0;
	tmp.y = 0.0;
	tmp.z = 0.0;
	tmp.normal_x = g_local[0];
	tmp.normal_y = g_local[1];
	tmp.normal_z = g_local[2];
	g_last->points.push_back(tmp);
}

void PointCloudFittingWalls::callback_cloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "-----CLOUD CALLBACK-----" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);

	if(g_local_is_available){
		pcl_visualization();
	}
}

void PointCloudFittingWalls::pcl_visualization(void)
{
	viewer.removePointCloud("g_last");
	viewer.addPointCloudNormals<pcl::PointNormal>(g_last, 1, 0.03, "g_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "g_last");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "g_last");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_fitting_walls");

	PointCloudFittingWalls pointcloudfittingwalls;

	ros::spin();
}
