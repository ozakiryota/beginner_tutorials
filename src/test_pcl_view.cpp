/*
 *	test_flann.cpp
 */

#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
// #include <flann/flann.hpp>
// #include <flann/io/hdf5.h>

void randomize_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud, int n)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<float> rnd_10(-10, 10);
	
	for(int i=0;i<n;i++){
		cloud.points[i].x = rnd_10(mt);
		cloud.points[i].y = rnd_10(mt);
		cloud.points[i].z = rnd_10(mt);
	}
	std::cout << "Randomized cloud" << std::endl;
}

void print_cloud(pcl::PointCloud<pcl::PointXYZ> cloud, int n)
{
	for(int i=0;i<n;i++)	std::cout << i+1 << ": " << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << std::endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_flann");
	ros::NodeHandle nh;
	
	const int num_points = 100;

	std::random_device rnd;
 	std::mt19937 mt(rnd());
 	std::uniform_real_distribution<float> rnd_10(-10, 10);

	pcl::visualization::CloudViewer viewer("Test Cloud Viewer");
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.points.resize(num_points);
	randomize_cloud(cloud, num_points);
	print_cloud(cloud, num_points);

	pcl::PointCloud<pcl::Normal> normal;

	while(ros::ok()){
		viewer.showCloud(cloud.makeShared());
	}
}
