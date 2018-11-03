#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>

class PCNormals{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		pcl::visualization::PCLVisualizer viewer{"pc_normals"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointNormal>::Ptr normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr extracted_normals {new pcl::PointCloud<pcl::PointNormal>};
		pcl::PointCloud<pcl::PointNormal>::Ptr gaussian_sphere {new pcl::PointCloud<pcl::PointNormal>};

	public:
		PCNormals();
		void Callback(const sensor_msgs::PointCloud2ConstPtr& msg);
		void GetGVector(void);
		void NormalEstimation(void);
		void GaussianSphere(void);
		void Visualizer(void);
};

PCNormals::PCNormals()
{
	sub = nh.subscribe("/velodyne_points", 1, &PCNormals::Callback, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.1, "axis");
}

void PCNormals::Callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	std::cout << "CALLBACK" << std::endl;
	pcl::fromROSMsg(*msg, *cloud);
	pcl::fromROSMsg(*msg, *normals);
	NormalEstimation();
	GaussianSphere();
	Visualizer();
}

void PCNormals::NormalEstimation(void)
{
	std::cout << "NORMAL ESTIMATION" << std::endl;
	pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
	ne.setInputCloud (normals);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.5);
	// for(size_t i=0;i<normals->points.size();i++)	normals->points[i].curvature = 0.0;
	ne.compute (*normals);

	extracted_normals->points.clear();
	for(size_t i=0;i<normals->points.size();i++){
		std::cout << "normals->points[i].curvature = " << normals->points[i].curvature << std::endl;
		if(normals->points[i].curvature<1.0e-7 && !(std::isnan(normals->points[i].curvature)))	extracted_normals->points.push_back(normals->points[i]);
	}
}

void PCNormals::GaussianSphere(void)
{
	std::cout << "GAUSSIAN SPHERE" << std::endl;
	// *gaussian_sphere = *normals;
	*gaussian_sphere = *extracted_normals;
	for(size_t i=0;i<gaussian_sphere->points.size();i++){
		gaussian_sphere->points[i].x = 0.0;
		gaussian_sphere->points[i].y = 0.0;
		gaussian_sphere->points[i].z = 0.0;
	}
}

void PCNormals::Visualizer(void)
{
	std::cout << "VISUALIZER" << std::endl;

	viewer.removePointCloud("cloud");
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

	viewer.removePointCloud("normals");
	viewer.addPointCloudNormals<pcl::PointNormal>(normals, 100, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");
	
	viewer.removePointCloud("extracted_normals");
	viewer.addPointCloudNormals<pcl::PointNormal>(extracted_normals, 100, 0.5, "extracted_normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "extracted_normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "extracted_normals");

	viewer.removePointCloud("gaussian_sphere");
	viewer.addPointCloudNormals<pcl::PointNormal>(gaussian_sphere, 1, 0.5, "gaussian_sphere");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "gaussian_sphere");
	

	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_normals");
	std::cout << "PC NORMALS" << std::endl;
	
	PCNormals pcnormals;
	ros::spin();
}
