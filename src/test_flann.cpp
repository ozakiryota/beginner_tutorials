/*
 *	test_flann.cpp
 */

#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


void randomize_points(std::vector<geometry_msgs::Point>& points, int n)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<float> rnd_10(-10, 10);

	for(int i=0;i<n;i++){
		geometry_msgs::Point p;
		p.x = rnd_10(mt);
		p.y = rnd_10(mt);
		p.z = rnd_10(mt);
		points.push_back(p);
	}
}

void print_points(std::vector<geometry_msgs::Point>& points, int n)
{
	for(int i=0;i<n;i++)	std::cout << i << ": " << points[i].x << "," << points[i].y << "," << points[i].z << std::endl;
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
	
	for(int i=0;i<num_points;i++){
		cloud.points[i].x = rnd_10(mt);
		cloud.points[i].y = rnd_10(mt);
		cloud.points[i].z = rnd_10(mt);
	}

	while(ros::ok()){
		viewer.showCloud(cloud.makeShared());
	}
}
