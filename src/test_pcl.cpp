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
	ne.setRadiusSearch(100);
	ne.compute(*normal);
	
	// std::cout << normal->points[0] << std::endl;
}

std::vector<int> kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius)
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
	// float radius = 1.0;
	kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	// if(kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
		for (size_t i=0;i<pointIdxRadiusSearch.size();i++){
			std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
			<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
			<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
	// }
	return pointIdxRadiusSearch; 
}

void plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud)
{
	float radius = 100.0;
	Eigen::Vector4f plane_parameters;
	std::vector<int> indices = kdtree_search(cloud, radius);
	float curvature;
	pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
	float sum_square_error = 0.0;
	for(int i=0;i<indices.size();i++){
		float square_error =	(plane_parameters[0]*cloud->points[indices[i]].x
								+plane_parameters[1]*cloud->points[indices[i]].y
								+plane_parameters[2]*cloud->points[indices[i]].z
								+plane_parameters[3])
								*(plane_parameters[0]*cloud->points[indices[i]].x
								+plane_parameters[1]*cloud->points[indices[i]].y
								+plane_parameters[2]*cloud->points[indices[i]].z
								+plane_parameters[3])
								/(plane_parameters[0]*plane_parameters[0]
								+plane_parameters[1]*plane_parameters[1]
								+plane_parameters[2]*plane_parameters[2]);
		sum_square_error += square_error/indices.size();
	}
	// normal->points[0].normal_x;
	// std::cout << sum_square_error << std::endl;
	// std::cout << "	" << plane_parameters << std::endl;
	
	planecloud->points[0] = cloud->points[0];
	normal->points[0].normal_x = plane_parameters[0];
	normal->points[0].normal_y = plane_parameters[1];
	normal->points[0].normal_z = plane_parameters[2];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_flann");
	ros::NodeHandle nh;
	
	const int num_points = 10;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(num_points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud (new pcl::PointCloud<pcl::PointXYZ>);
	planecloud->points.resize(num_points);
	randomize_cloud(cloud, num_points);
	print_cloud(cloud, num_points);
	pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
	normal->points.resize(num_points);
	// estimate_normal(cloud, normal);

	// kdtree_search(cloud);
	Eigen::Vector4f plane_parameters;
	plane_fitting(cloud, normal, planecloud);

	pcl::visualization::PCLVisualizer viewer("Test Point Cloud Viewer");
	viewer.addPointCloud(cloud, "cloud");
	// viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 1, 0.3, "normals");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(planecloud, normal, 1, 0.5, "normals");
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 5, 0.3, "normals");

	// std::cout << "TEST" << std::endl;
	while(ros::ok()){
		viewer.spinOnce();
		// viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
