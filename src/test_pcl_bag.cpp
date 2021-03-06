/*
 *	test_pcl_bag.cpp
 */

#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 tmp;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "cloud_callback" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *tmp_cloud);
	*cloud += *tmp_cloud;

	std::cout << *cloud << std::endl;
	for (int i=0;i<10;i++)	std::cout << i << ":" << cloud->points[i] << std::endl;
}

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

std::vector<int> kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius, pcl::PointXYZ searchpoint)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	// if(kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
	// 	for (size_t i=0;i<pointIdxRadiusSearch.size();i++){
	// 		std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
	// 		<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
	// 		<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
	// 		<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	// 	}
	// }
	return pointIdxRadiusSearch; 
}

void plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud)
{
	const float radius = 0.5;
	float curvature;
	int num_normals = 0;
	for(int i=0;i<cloud->points.size();i+=10000){
		pcl::PointXYZ searchpoint;
		searchpoint.x = cloud->points[i].x;
		searchpoint.y = cloud->points[i].y;
		searchpoint.z = cloud->points[i].z;
		std::vector<int> indices = kdtree_search(cloud, radius, searchpoint);

		Eigen::Vector4f plane_parameters;
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
			sum_square_error += square_error/(float)indices.size();
		}
		std::cout << "indices.size() = " << indices.size() << std::endl;
		std::cout << "sum_square_error = " << sum_square_error << std::endl;
		std::cout << "curvature = " << plane_parameters[3] << std::endl;
		if(sum_square_error<0.01&&plane_parameters[2]<0.8){
			planecloud->points[num_normals] = cloud->points[i];
			normal->points[num_normals].normal_x = plane_parameters[0];
			normal->points[num_normals].normal_y = plane_parameters[1];
			normal->points[num_normals].normal_z = plane_parameters[2];
			num_normals++;
		}
	}
	for(int i=0;i<num_normals;i++)	std::cout << normal->points[i] << std::endl;
}

void estimate_g_vector(pcl::PointCloud<pcl::Normal>::Ptr normal, pcl::PointCloud<pcl::PointXYZ>::Ptr g_point, pcl::PointCloud<pcl::Normal>::Ptr g_vector)
{
	pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
	tmp_cloud.points.resize(normal->points.size());
	for(int i=0;i<normal->points.size();i++){
		tmp_cloud.points[i].x = normal->points[i].normal_x;
		tmp_cloud.points[i].y = normal->points[i].normal_y;
		tmp_cloud.points[i].z = normal->points[i].normal_z;
	}
	Eigen::Vector4f g_parameters;
	float curvature; 
	pcl::computePointNormal(tmp_cloud, g_parameters, curvature);
	std::cout << "gravity" << std::endl << g_parameters << std::endl;
	g_vector->points[0].normal_x = g_parameters[0];
	g_vector->points[0].normal_y = g_parameters[1];
	g_vector->points[0].normal_z = g_parameters[2];
	g_point->points[0].x = 0;
	g_point->points[0].y = 0;
	g_point->points[0].z = 0;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_pcl_bag");
	ros::NodeHandle nh;

	ros::Subscriber cloud_sub = nh.subscribe("/cloud", 20, cloud_callback);

	// const int num_points = 10000;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::io::loadPCDFile ("/home/amsl/ros_catkin_ws/src/beginner_tutorials/map_0.pcd", *cloud);
	// int num_points = cloud->points.size();
	// cloud->points.resize(num_points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud (new pcl::PointCloud<pcl::PointXYZ>);
	planecloud->points.resize(cloud->points.size());
	// randomize_cloud(cloud, num_points);
	print_cloud(cloud, cloud->points.size());
	pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
	normal->points.resize(cloud->points.size());
	// estimate_normal(cloud, normal);

	Eigen::Vector4f plane_parameters;
	plane_fitting(cloud, normal, planecloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr g_point (new pcl::PointCloud<pcl::PointXYZ>);
	g_point->points.resize(1);
	pcl::PointCloud<pcl::Normal>::Ptr g_vector (new pcl::PointCloud<pcl::Normal>);
	g_vector->points.resize(1);
	// estimate_g_vector(normal, g_point, g_vector);

	pcl::visualization::PCLVisualizer viewer("Test Point Cloud Viewer");
	viewer.addPointCloud(cloud, "cloud");
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(planecloud, normal, 1, 0.5, "normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals"); 
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals"); 
	
	viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(g_point, g_vector, 1, 0.5, "g");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "g");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "g"); 

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normal, 5, 0.3, "normals");

	// std::cout << "TEST" << std::endl;
	int roop_count = 0;
	int stock = 1000000;
	while(ros::ok()){
		roop_count %= stock;
		roop_count += 1;
		// std::cout << roop_count << std::endl;
		viewer.spinOnce();
		if(roop_count==stock){
			viewer.removePointCloud("cloud");
			viewer.addPointCloud(cloud, "cloud");
			cloud->points.erase(cloud->points.begin(), cloud->points.begin()+cloud->points.size());
		}
		// viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		ros::spinOnce();
	}
}
