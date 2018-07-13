/*
 *	test_pcl_kdtree.cpp
 */

#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
// #include <flann/flann.hpp>
// #include <flann/io/hdf5.h>

void randomize_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<float> rnd_10(-10, 10);
	
	for(int i=0;i<n;i++){
		cloud->points[i].x = rnd_10(mt);
		cloud->points[i].y = rnd_10(mt);
		cloud->points[i].z = rnd_10(mt);
	}
	std::cout << "Randomized cloud" << std::endl;
}

void print_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n)
{
	for(int i=0;i<n;i++)	std::cout << i+1 << ": " << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << std::endl;
}

void estimate_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(5);
	ne.compute(*normal);
}

void kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	pcl::PointXYZ searchpoint;
	int cloud_index = 0;
	searchpoint.x = cloud->points[cloud_index].x;
	searchpoint.y = cloud->points[cloud_index].y;
	searchpoint.z = cloud->points[cloud_index].z;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 1.0;

	if(kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
		for (size_t i=0;i<pointIdxRadiusSearch.size();i++){
			std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
			<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
			<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_flann");
	ros::NodeHandle nh;
	
	const int num_points = 1000;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(num_points);
	randomize_cloud(cloud, num_points);
	print_cloud(cloud, num_points);
	pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
	estimate_normal(cloud, normal);

	kdtree_search(cloud);

	pcl::visualization::PCLVisualizer viewer("Test Point Cloud Viewer");
	viewer.addPointCloud(cloud, "cloud");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 10, 0.3, "normals");
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 5, 0.3, "normals");

	while(ros::ok()){
		viewer.spinOnce();
		// viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
