/*
 *	pc_normal.cpp
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
// sensor_msgs::PointCloud2 tmp;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "cloud_callback" << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg, *tmp_cloud);
	*cloud += *tmp_cloud;
	if(cloud->points.size()>10*tmp_cloud->points.size())	cloud->points.erase(cloud->points.begin(), cloud->points.begin()+tmp_cloud->points.size());
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

void plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud, std::vector<float>& fitting_errors)
{
	const float radius = 0.5;
	float curvature;
	int num_normals = 0;
	// std::vector<float> fitting_errors;
	for(int i=0;i<cloud->points.size();i+=10000){
		pcl::PointXYZ searchpoint;
		searchpoint.x = cloud->points[i].x;
		searchpoint.y = cloud->points[i].y;
		searchpoint.z = cloud->points[i].z;
		std::vector<int> indices = kdtree_search(cloud, radius, searchpoint);

		Eigen::Vector4f plane_parameters;
		// setViewPoint (float vpx, float vpy, float vpz);
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		if(plane_parameters[2]>0.8)	continue;
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
		if(sum_square_error<0.01){
			planecloud->points[num_normals] = cloud->points[i];
			normals->points[num_normals].normal_x = plane_parameters[0];
			normals->points[num_normals].normal_y = plane_parameters[1];
			normals->points[num_normals].normal_z = plane_parameters[2];
			fitting_errors.push_back(sum_square_error);
			num_normals++;
		}
	}
	for(int i=0;i<num_normals;i++)	std::cout << normals->points[i] << std::endl;
}

void estimate_g_vector(pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr g_point, pcl::PointCloud<pcl::Normal>::Ptr g_vector)
{
	pcl::PointCloud<pcl::PointXYZ> normal_sphere;
	normal_sphere.points.resize(normals->points.size());
	for(int i=0;i<normals->points.size();i++){
		normal_sphere.points[i].x = normals->points[i].normal_x;
		normal_sphere.points[i].y = normals->points[i].normal_y;
		normal_sphere.points[i].z = normals->points[i].normal_z;
	}
	Eigen::Vector4f g_parameters;
	float curvature; 
	pcl::computePointNormal(normal_sphere, g_parameters, curvature);
	std::cout << "gravity" << std::endl << g_parameters << std::endl;
	g_vector->points[0].normal_x = g_parameters[0];
	g_vector->points[0].normal_y = g_parameters[1];
	g_vector->points[0].normal_z = g_parameters[2];
	g_point->points[0].x = 0;
	g_point->points[0].y = 0;
	g_point->points[0].z = 0;
}

void reset(void)
{
	//Do I need to reset anything during the cycle?
}

void clustering(pcl::PointCloud<pcl::Normal>::Ptr normals, std::vector<float> fitting_errors)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere;
	normal_sphere->points.resize(normals->points.size());
	for(int i=0;i<normals->points.size();i++){
		normal_sphere->points[i].x = normals->points[i].normal_x;
		normal_sphere->points[i].y = normals->points[i].normal_y;
		normal_sphere->points[i].z = normals->points[i].normal_z;
	}
	std::vector<int> num_members(normal_sphere->points.size(), 1);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	int k = 1;
	std::vector<int> pointIdxNKNSearch(k);
	std::vector<float> pointNKNSquaredDistance(k);
	pcl::PointXYZ searchpoint;
	// std::vector<int> nearest_index_list;
	// std::vector<float> nearest_distance_list;
	// float shortest_index = 0;
	// float shortest_distance;

	/*
	kdtree.setInputCloud(normal_sphere);
	for(int i=0;i<normals->points.size();i++){
		searchpoint = normal_sphere.points[i];
		if(kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance)<0)	break;
		nearest_index_list.push_back(pointIdxNKNSearch[0]);
		nearest_distance_list.push_back(pointNKNSquaredDistance[0]);
		if(i==0)	shortest_distance = pointNKNSquaredDistance[0];
		else if(pointNKNSquaredDistance[0]<shortest_distance){
			shortest_index = i;
			shortest_distance = pointNKNSquaredDistance[0];
		}
	}
	const float fitting_error_threshold = 0.01;
	while(ros::ok()){
		std::vector<float> n1(3) = {normal_sphere.points[shortest_index].x, normal_sphere.points[shortest_index].y, normal_sphere.points[shortest_index].z}
		float w1 = fitting_error_threshold/fitting_error[shortest_index];
		std::vector<float> n2(3) = {normal_sphere.points[pointIdxNKNSearch[shortest_index]].x, normal_sphere.points[pointIdxNKNSearch[shortest_index]].y, normal_sphere.points[pointIdxNKNSearch[shortest_index]].z}
		float w2 = fitting_error_threshold/fitting_error[pointIdxNKNSearch[shortest_index]];
		
		normal_spere.points[shortest_index].x = (n1[0]*w1 + n2[0]*w2)/(w1 + w2);
		normal_spere.points[shortest_index].y = (n1[1]*w1 + n2[1]*w2)/(w1 + w2);
		normal_spere.points[shortest_index].z = (n1[2]*w1 + n2[2]*w2)/(w1 + w2);
		fitting_error[shortest_index] = w1*w2;
			
		normal_sphere->points.erase(normal_sphere->points.begin()+pointIdxNKNSearch[shortest_index]);
		
		
		kdtree.setInputCloud(normal_sphere);
		searchpoint = normal_sphere.points[i];

		if(nearest_distance>0.1)	break;
	}
	*/
	
	const float fitting_error_threshold = 0.01;
	while(ros::ok()){
		kdtree.setInputCloud(normal_sphere);
		// std::vector<int> nearest_index_list;
		// std::vector<float> nearest_distance_list;
		float shortest_dist_index = 0;
		float shortest_distance;
		for(int i=0;i<normal_sphere->points.size();i++){
			searchpoint = normal_sphere->points[i];
			if(kdtree.nearestKSearch(searchpoint, k, pointIdxNKNSearch, pointNKNSquaredDistance)<0){
				std::cout << "error" << std::endl;
				break;
			}
			// nearest_index_list.push_back(pointIdxNKNSearch[0]);
			// nearest_distance_list.push_back(pointNKNSquaredDistance[0]);
			if(i==0)	shortest_distance = pointNKNSquaredDistance[0];
			else if(pointNKNSquaredDistance[0]<shortest_distance){
				shortest_dist_index = i;
				shortest_distance = pointNKNSquaredDistance[0];
			}
		}
		if(shortest_distance>0.01) break;
		std::vector<float> n1 = {normal_sphere->points[shortest_dist_index].x, normal_sphere->points[shortest_dist_index].y, normal_sphere->points[shortest_dist_index].z};
		float w1 = fitting_error_threshold/fitting_errors[shortest_dist_index] * num_members[shortest_dist_index]/(float)normals->points.size();
		std::vector<float> n2 = {normal_sphere->points[pointIdxNKNSearch[shortest_dist_index]].x, normal_sphere->points[pointIdxNKNSearch[shortest_dist_index]].y, normal_sphere->points[pointIdxNKNSearch[shortest_dist_index]].z};
		float w2 = fitting_error_threshold/fitting_errors[pointIdxNKNSearch[shortest_dist_index]] * num_members[pointIdxNKNSearch[shortest_dist_index]]/(float)normals->points.size();
		
		normal_sphere->points[shortest_dist_index].x = (n1[0]*w1 + n2[0]*w2)/(w1 + w2);
		normal_sphere->points[shortest_dist_index].y = (n1[1]*w1 + n2[1]*w2)/(w1 + w2);
		normal_sphere->points[shortest_dist_index].z = (n1[2]*w1 + n2[2]*w2)/(w1 + w2);
		fitting_errors[shortest_dist_index] = (fitting_errors[shortest_dist_index] + fitting_errors[pointIdxNKNSearch[shortest_dist_index]])/2.0;
		num_members[shortest_dist_index] += num_members[pointIdxNKNSearch[shortest_dist_index]];
		
		normal_sphere->points.erase(normal_sphere->points.begin()+pointIdxNKNSearch[shortest_dist_index]);
		fitting_errors.erase(fitting_errors.begin()+pointIdxNKNSearch[shortest_dist_index]);
		num_members.erase(num_members.begin()+pointIdxNKNSearch[shortest_dist_index]);
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_normal");
	ros::NodeHandle nh;

	/*pub sub*/
	ros::Subscriber cloud_sub = nh.subscribe("/cloud", 20, cloud_callback);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/cloud",1);

	// const int num_points = 10000;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::io::loadPCDFile ("/home/amsl/ros_catkin_ws/src/beginner_tutorials/map_0.pcd", *cloud);
	// int num_points = cloud->points.size();
	// cloud->points.resize(num_points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud (new pcl::PointCloud<pcl::PointXYZ>);
	planecloud->points.resize(cloud->points.size());
	// randomize_cloud(cloud, num_points);
	print_cloud(cloud, cloud->points.size());
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	normals->points.resize(cloud->points.size());
	// estimate_normal(cloud, normals);

	// Eigen::Vector4f plane_parameters;
	std::vector<float> fitting_errors;
	plane_fitting(cloud, normals, planecloud, fitting_errors);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr g_point (new pcl::PointCloud<pcl::PointXYZ>);
	g_point->points.resize(1);
	pcl::PointCloud<pcl::Normal>::Ptr g_vector (new pcl::PointCloud<pcl::Normal>);
	g_vector->points.resize(1);
	// estimate_g_vector(normals, g_point, g_vector);

	// pcl::visualization::PCLVisualizer viewer("Test Point Cloud Viewer");
	// viewer.addPointCloud(cloud, "cloud");
	// viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(planecloud, normals, 1, 0.5, "normals");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals"); 
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals"); 
	//
	// viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(g_point, g_vector, 1, 0.5, "g");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "g");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "g"); 

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	// viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 5, 0.3, "normals");

	// std::cout << "TEST" << std::endl;
	int roop_count = 0;
	int stock = 1000000;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		roop_count %= stock;
		roop_count += 1;
		// std::cout << roop_count << std::endl;
		// viewer.spinOnce();
		// if(roop_count==stock){
		// 	viewer.removePointCloud("cloud");
		// 	viewer.addPointCloud(cloud, "cloud");
		// 	cloud->points.erase(cloud->points.begin(), cloud->points.begin()+cloud->points.size());
		// }
		if(!cloud->points.empty()){
			sensor_msgs::PointCloud2 roscloud_out;
			// roscloud_out.header.frame_id = "/centerlaser_";
			pcl::toROSMsg(*cloud, roscloud_out);
			roscloud_out.header.frame_id = "/centerlaser_";
			cloud_pub.publish(roscloud_out);
		}
		// viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		ros::spinOnce();
		loop_rate.sleep();
	}
}
