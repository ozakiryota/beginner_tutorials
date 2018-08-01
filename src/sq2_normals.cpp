/*
 *	pc_normal.cpp
 */

#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
// std::vector<int> num_subpoints;
// ros::Time tm;
std::vector<float> g_est ={
	0.0,
	0.0,
	1.0};

struct	FEATURES{
	size_t num_refpoints;
	float fitting_error;
	float ang_from_g_est;
	float weight;
	int neighbor_index;
	int num_groups;
};

/*-----getParam-----*/
float LOOP_RATE;
float SEARCH_RADIUS;
int RANDOM_STEP_MAX;
float THRESHOLD_ANGLE_FROM_G;
float THRESHOLD_SUM_SQUARE_ERRORS;
float MIN_DISTANCE_BETWEEN_NORMSLS;
// std::vector<float> PARAMETERS_FOR_FEATURES;
float FACTOR_1;
float FACTOR_2;
float FACTOR_3;
float FACTOR_4;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	// std::cout << "cloud_callback" << std::endl;
	// tm = msg->header.stamp;
	pcl::fromROSMsg(*msg, *cloud);
	// std::cout << "msg->data.size() = " << msg->data.size() << std::endl;


	// pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::fromROSMsg(*msg, *tmp_cloud);
	// for(int i=0;i<tmp_cloud->points.size();i++)	cloud->points.push_back(tmp_cloud->points[i]);
	// num_subpoints.push_back(tmp_cloud->points.size());
	// if(num_subpoints.size()==100){
	// 	cloud->points.erase(cloud->points.begin(), cloud->points.begin()+num_subpoints[0]);
	// 	num_subpoints.erase(num_subpoints.begin());
	// }
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

void randomize_normals(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, int n)
{
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_real_distribution<float> rnd_10(-10, 10);
	
	normals->points.resize(n);
	for(int i=0;i<n;i++){
		normals->points[i].x = rnd_10(mt);
		normals->points[i].z = rnd_10(mt);
		normals->points[i].y = rnd_10(mt);
		normals->points[i].normal_x = rnd_10(mt);
		normals->points[i].normal_y = rnd_10(mt);
		normals->points[i].normal_z = rnd_10(mt);
	}
	std::cout << "Randomized normals" << std::endl;
}


void print_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n)
{
	for(int i=0;i<n;i++)	std::cout << i+1 << ": " << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << std::endl;
}

// std::vector<int> kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius, pcl::PointXYZ searchpoint)
std::vector<int> kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ searchpoint)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	// kdtree.radiusSearch(searchpoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
	kdtree.radiusSearch(searchpoint, SEARCH_RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance);
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

float angle_between_vectors(std::vector<float> v1, std::vector<float> v2)
{
	std::cout << "-ANGLE BETWEEN VECTORS-" << std::endl;
	if(v1.size()!=v2.size()){
		std::cout << "error: v1.size()!=v2.size()" << std::endl;
		exit(1);
		return -1;
	}
	float dot_product = 0.0;
	float v1_norm = 0.0;
	float v2_norm = 0.0;
	for(size_t i=0;i<v1.size();i++){
		dot_product += v1[i]*v2[i];
		v1_norm += v1[i]*v1[i];
		v2_norm += v2[i]*v2[i];
	}
	v1_norm = sqrt(v1_norm);
	v2_norm = sqrt(v2_norm);
	// std::cout << "return angle" << std::endl;
	return fabs(acosf(dot_product/(v1_norm*v2_norm)));
}

// void plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr planecloud, std::vector<float>& fitting_errors)
// void plane_fitting(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, std::vector<float>& fitting_errors, std::vector<int>& num_refpoints)
void plane_fitting(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, std::vector<FEATURES>& features)
{
	std::cout << "-----PLANE FITTING-----" <<  std::endl;
	// const float radius = 0.5;
	float curvature;
	// int num_normals = 0;
	// std::vector<float> fitting_errors;
	std::random_device rnd;
	std::mt19937 mt(rnd());
	// const int step_max = 200;
	// std::uniform_int_distribution<> rand_step(1, step_max);
	std::uniform_int_distribution<> rand_step(1, RANDOM_STEP_MAX);
	// for(int i=0;i<cloud->points.size();i+=1000){
	int i = -1;
	// int step = rand1000(mt);
	// std::cout << "step = " << step << std::endl;
	// while(i<cloud->points.size()){
	while(ros::ok()){
	// for(int i=0;i<cloud->points.size();i+=1000){
		i += rand_step(mt);
		// for(int i=0;i<cloud->points.size();i+=100){
		// std::cout << i << std::endl;
		if(i>=cloud->points.size())	break;
		// std::cout << "loop" << std::endl;
		pcl::PointXYZ searchpoint;
		searchpoint.x = cloud->points[i].x;
		searchpoint.y = cloud->points[i].y;
		searchpoint.z = cloud->points[i].z;
		// std::cout << "start finding kdtree" <<  std::endl;
		// if(searchpoint.z<0.3){
		// 	std::cout << ">> searchpoint.z<0.5, then skip" << std::endl;
		// 	continue;
		// }
		std::vector<int> indices = kdtree_search(cloud, searchpoint);
		std::cout << "indices.size() = " << indices.size() << std::endl;
		if(indices.size()<3){
			std::cout << ">> indices.size()<3, then skip" << std::endl;
			continue;
		}
		Eigen::Vector4f plane_parameters;
		// setViewPoint (float vpx, float vpy, float vpz);
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		// std::cout << "norm of normal = " << sqrt(plane_parameters[0]*plane_parameters[0] + plane_parameters[1]*plane_parameters[1] + plane_parameters[2]*plane_parameters[2]) << std::endl;
		
		float tmp_ang_from_g_est = fabs(acosf(plane_parameters[0]*g_est[0] + plane_parameters[1]*g_est[1] + plane_parameters[2]*g_est[2]));
		// std::coutimate a covariance matrix from a set of points in PCL, you can use: << "tmp_ang_from_g_est = " << tmp_ang_from_g_est << std::endl;
		// std::vector<float> a = {plane_parameters[0], plane_parameters[1], plane_parameters[2]};
		// tmp_ang_from_g_est = angle_between_vectors(a, g_est);
		// std::cout << "tmp_ang_from_g_est = " << tmp_ang_from_g_est << std::endl;

		// std::cout << "fabs(tmp_ang_from_g_est) = " << fabs(tmp_ang_from_g_est) << std::endl;
		// const float ang_threshold = 30.0/180.0*M_PI;
		// std::cout << "ang_threshold = " << ang_threshold << std::endl;
		std::cout << "THRESHOLD_ANGLE_FROM_G = " << THRESHOLD_ANGLE_FROM_G << std::endl;
		if(tmp_ang_from_g_est<THRESHOLD_ANGLE_FROM_G/180.0*M_PI || tmp_ang_from_g_est>(M_PI-THRESHOLD_ANGLE_FROM_G/180.0*M_PI)){
			std::cout << ">> tmp_ang_from_g_est is too small or large, then skip" << std::endl;
			continue;
		}
		//if(indices.size()<3||plane_parameters[2]>0.8||plane_parameters[2]<-0.8){
			// std::cout << "continue" << std::endl;
			// continue;
		// }
		float sum_square_error = 0.0;
		// std::cout << "start caluculating square_error" <<  std::endl;
		for(int j=0;j<indices.size();j++){
			float square_error =	(plane_parameters[0]*cloud->points[indices[j]].x
									+plane_parameters[1]*cloud->points[indices[j]].y
									+plane_parameters[2]*cloud->points[indices[j]].z
									+plane_parameters[3])
									*(plane_parameters[0]*cloud->points[indices[j]].x
									+plane_parameters[1]*cloud->points[indices[j]].y
									+plane_parameters[2]*cloud->points[indices[j]].z
									+plane_parameters[3])
									/(plane_parameters[0]*plane_parameters[0]
									+plane_parameters[1]*plane_parameters[1]
									+plane_parameters[2]*plane_parameters[2]);
			sum_square_error += square_error/(float)indices.size();
		}
		// std::cout << "indices.size() = " << indices.size() << std::endl;
		std::cout << "sum_square_error = " << sum_square_error << std::endl;
		// std::cout << "curvature = " << plane_parameters[3] << std::endl;
		// const float sum_square_error_threshold = 0.1;
		if(sum_square_error>THRESHOLD_SUM_SQUARE_ERRORS)	std::cout << ">> sum_square_error is too large, then skip" << std::endl;
		if(sum_square_error<THRESHOLD_SUM_SQUARE_ERRORS){
			// planecloud->points[num_normals] = cloud->points[i];
			pcl::PointXYZINormal tmp_normal;
			tmp_normal.x = cloud->points[i].x;
			tmp_normal.y = cloud->points[i].y;
			tmp_normal.z = cloud->points[i].z;
			flipNormalTowardsViewpoint (tmp_normal, 0.0, 0.0, 0.1, plane_parameters);
			tmp_normal.normal_x = plane_parameters[0];
			tmp_normal.normal_y = plane_parameters[1];
			tmp_normal.normal_z = plane_parameters[2];
			normals->points.push_back(tmp_normal);
			// fitting_errors.push_back(sum_square_error);
			// num_refpoints.push_back(indices.size());
			features.push_back({
				indices.size(),
				sum_square_error,
				tmp_ang_from_g_est,
				indices.size()/sum_square_error,
				i,
				1});
			// std::cout << "features[features.size()-1].num_refpoints = " << features[features.size()-1].num_refpoints << std::endl;
			// num_normals++;
		}
	}
	// for(int i=0;i<normals->points.size();i++)	std::cout << normals->points[i] << std::endl;
	// int step = rand1000(mt);
	// i += step;
}
	// features[shortest_dist_index].fitting_errors[shortest_dist_index] = (fitting_errors[shortest_dist_index] + fitting_errors[pointIdxNKNSearch[shortest_dist_index]])/2.0;

// bool estimate_g_vector(pcl::PointCloud<pcl::PointXYZINormal>::Ptr main_normals, pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_vector)
bool estimate_g_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere, pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_vector)
{
	std::cout << "-----ESTIMATE G_VECTOR-----" << std::endl;
	if(normal_sphere->points.size()<2){
		std::cout << "Failed because normal_sphere has less than 2 normals" << std::endl;
		return false;
	}
	// pcl::PointCloud<pcl::PointXYZ> normal_sphere;
	// normal_sphere.points.resize(main_normals->points.size());
	// for(int i=0;i<main_normals->points.size();i++){
	//	normal_sphere.points[i].x = main_normals->points[i].normal_x;
	//	normal_sphere.points[i].y = main_normals->points[i].normal_y;
	//	normal_sphere.points[i].z = main_normals->points[i].normal_z;
	// }
	g_vector->points[0].x = 0;
	g_vector->points[0].y = 0;
	g_vector->points[0].z = 1.0;
	if(normal_sphere->points.size()==2){
		g_vector->points[0].normal_x = normal_sphere->points[0].y*normal_sphere->points[1].z - normal_sphere->points[0].z*normal_sphere->points[1].y;
		g_vector->points[0].normal_y = normal_sphere->points[0].z*normal_sphere->points[1].x - normal_sphere->points[0].x*normal_sphere->points[1].z;
		g_vector->points[0].normal_z = normal_sphere->points[0].x*normal_sphere->points[1].y - normal_sphere->points[0].y*normal_sphere->points[1].x;
		std::cout << "g_vector->points[0] = " << g_vector->points[0] << std::endl;
		return true;
	}
	Eigen::Vector4f g_parameters;
	float curvature; 
	pcl::computePointNormal(*normal_sphere, g_parameters, curvature);
	// pcl::computePointNormal(*main_normals, g_parameters, curvature);
	// std::cout << "gravity" << std::endl << g_parameters << std::endl;
	g_vector->points[0].normal_x = -g_parameters[0];
	g_vector->points[0].normal_y = -g_parameters[1];
	g_vector->points[0].normal_z = -g_parameters[2];
	std::cout << "g_vector->points[0] = " << g_vector->points[0] << std::endl;
	return true;
}

void reset(void)
{
	//Do I need to reset anything during the cycle?
}

void merge_vectors__(std::vector<float>& n1, std::vector<float> n2, float w1, float w2)
{
	std::vector<float> n1_ = {
		acosf(1.0/n1[0]),
		acosf(1.0/n1[1]),
		acosf(1.0/n1[2])};
	std::vector<float> n2_ = {
		acosf(1.0/n2[0]),
		acosf(1.0/n2[1]),
		acosf(1.0/n2[2])};
	n1[0] = cos(w1*n1_[0] + w2*n2_[0])/(w1 + w2);
	n1[1] = cos(w1*n1_[1] + w2*n2_[1])/(w1 + w2);
	n1[2] = cos(w1*n1_[2] + w2*n2_[2])/(w1 + w2);
}

std::vector<float> merge_vectors(std::vector<float> v1, std::vector<float> v2, float w1, float w2)
{
	std::cout << "---MERGE VECTORS---" << std::endl;
	std::vector<float> v;
	float norm = 0.0;
	for(size_t i=0;i<v1.size();i++){
		// v.push_back(w1*v1[i] + w2*v2[i]);
		v.push_back((w1*v1[i] + w2*v2[i])/(w1 + w2));
		norm += v[i]*v[i];
	}
	norm = sqrt(norm);
	// std::cout << "norm = " << norm << std::endl;
	if(norm==0.0){
		std::cout << "norm == 0.0" << std::endl;
		exit(1);
	}
	for(size_t i=0; i<v1.size();i++)	v[i] /= norm;
	return v;
}

void merge_vectors_(std::vector<float>& n1, std::vector<float> n2, float w1, float w2)
{
	std::cout << "-----MERGE VECTORS-----" << std::endl;
	float norm = 0.0;
	for(size_t i=0; i<n1.size();i++){
		n1[i] = w1*n1[i] + w2*n2[i];
		norm += n1[i]*n1[i];
	}
	norm = sqrt(norm);
	for(size_t i=0; i<n1.size();i++)	n1[i] /= norm;
}

// void clustering(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, std::vector<float> fitting_errors, std::vector<int> num_refpoints)
// void clustering(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, std::vector<FEATURES> features)
void clustering(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, std::vector<FEATURES> features, pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere)
{
	std::cout << "-----CLUSTERING-----" << std::endl;
	std::cout << "normals->points.size() = " << normals->points.size() << std::endl;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere (new pcl::PointCloud<pcl::PointXYZ>);
	// normal_sphere->points.resize(normals->points.size());
	// std::vector<float> weight;
	
	// for(int i=0;i<normals->points.size();i++){
		// normal_sphere->points[i].x = normals->points[i].normal_x;
		// normal_sphere->points[i].y = normals->points[i].normal_y;
		// normal_sphere->points[i].z = normals->points[i].normal_z;
	//	normal_sphere->points.push_back({normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z});
	// }
	
	// std::vector<int> num_members(normal_sphere->points.size(), 1);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	int k = 2;
	// std::vector<int> pointIdxNKNSearch(k);
	// std::vector<float> pointNKNSquaredDistance(k);
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
		std::vector<float> n1(3) = {normal_sphere.points[shortest_index].x, normal_sphere.points[shorstd::auto_ptr<CloudViewer_impl> impl_;test_index].y, normal_sphere.points[shortest_index].z}
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
	
	// const float fitting_error_threshold = 0.01;
	while(ros::ok()){
		// for(int j=0;j<normal_sphere->points.size();j++){
		// 	std::cout << j << ": normal_sphere->points[j] = " << normal_sphere->points[j] << std::endl;
		// 	std::cout << "   features[j].num_refpoints = " << features[j].num_refpoints << std::endl;
		// }
		if(normal_sphere->points.size()<k){
			std::cout << "= END MERGE =" << std::endl;
			break;
		}
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		kdtree.setInputCloud(normal_sphere);
		// std::vector<int> nearest_index_list;
		// std::vector<float> nearest_distance_list;
		float shortest_dist_index;	//A pair of points[shortest_dist_index] and points[features[neighbor_index]] is the closest
		float shortest_distance;
		for(int i=0;i<normal_sphere->points.size();i++){
			searchpoint = normal_sphere->points[i];
			if(kdtree.nearestKSearch(searchpoint, k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0){
				std::cout << "error" << std::endl;
				// exit(1);
				break;
			}
			// std::cout << "pointIdxNKNSearch = " << pointIdxNKNSearch[0] << std::endl;
			features[i].neighbor_index = pointIdxNKNSearch[1];
			// std::cout << i << ":features[i].neighbor_index = " << features[i].neighbor_index << std::endl;
			// nearest_index_list.push_back(pointIdxNKNSearch[0]);
			// nearest_distance_list.push_back(pointNKNSquaredDistance[0]);
			
			std::cout << i << ": normal_sphere->points[i] = " << normal_sphere->points[i] << std::endl;
			std::cout << "   features[i].num_refpoints = " << features[i].num_refpoints << std::endl;
			// std::cout << "   features[i].fitting_error = " << features[i].fitting_error << std::endl;
			std::cout << "   1/features[i].fitting_error = " << 1/features[i].fitting_error << std::endl;
			// std::cout << "   features[i].ang_from_g_est = " << features[i].ang_from_g_est << std::endl;
			std::cout << "   1/fabs(M_PI/2.0 - features[i].ang_from_g_est) = " << 1/fabs(M_PI/2.0 - features[i].ang_from_g_est) << std::endl;
			std::cout << "   features[i].num_groups = " << features[i].num_groups << std::endl;
			std::cout << "   features[i].weight = " << features[i].weight << std::endl;
			std::cout << "   pointIdxNKNSearch[1] = " << pointNKNSquaredDistance[1] << std::endl;

			if(i==0){
				shortest_dist_index = 0;
				shortest_distance = pointNKNSquaredDistance[1];
			}
			else if(pointNKNSquaredDistance[1]<shortest_distance){
				shortest_dist_index = i;
				shortest_distance = pointNKNSquaredDistance[1];
			}
		}



		// std::cout << "shortest_dist_index = " << shortest_dist_index << std::endl;
		std::cout << "shortest_distance = " << shortest_distance << std::endl;
		// const float shortest_distance_threshold = 0.3;
		// if(shortest_distance>shortest_distance_threshold){
		if(shortest_distance>MIN_DISTANCE_BETWEEN_NORMSLS){
			std::cout << "= END MERGE =" << std::endl;
			break;
		}
		std::cout << "MERGE " << shortest_dist_index << " & " << features[shortest_dist_index].neighbor_index << std::endl;
		std::vector<float> n1 = {
			normal_sphere->points[shortest_dist_index].x,
			normal_sphere->points[shortest_dist_index].y,
			normal_sphere->points[shortest_dist_index].z};
		float w1 = features[shortest_dist_index].weight;
		// float w1 = fitting_error_threshold/fitting_errors[shortest_dist_index] * num_members[shortest_dist_index]/(float)normals->points.size();
		std::vector<float> n2 = {
			// normal_sphere->points[pointIdxNKNSearch[shortest_dist_index]].x,
			// normal_sphere->points[pointIdxNKNSearch[shortest_dist_index]].y,
			// normal_sphere->points[pointIdxNKNSearch[shortest_dist_index]].z
			normal_sphere->points[features[shortest_dist_index].neighbor_index].x,
			normal_sphere->points[features[shortest_dist_index].neighbor_index].y,
			normal_sphere->points[features[shortest_dist_index].neighbor_index].z};
		float w2 = features[features[shortest_dist_index].neighbor_index].weight;
		// float w2 = fitting_error_threshold/fitting_errors[pointIdxNKNSearch[shortest_dist_index]] * num_members[pointIdxNKNSearch[shortest_dist_index]]/(float)normals->points.size();
		std::cout << "w1 = " << w1 << std::endl;
		std::cout << "w2 = " << w2 << std::endl;
		std::cout << "w1/(w1 + w2) = " << w1/(w1 + w2) << std::endl;

		// std::cout << "merge" << std::endl;
		std::vector<float> merged_n = merge_vectors(n1, n2, w1, w2);
		normal_sphere->points[shortest_dist_index].x = merged_n[0];
		normal_sphere->points[shortest_dist_index].y = merged_n[1];
		normal_sphere->points[shortest_dist_index].z = merged_n[2];
		// normal_sphere->points[shortest_dist_index].x = (n1[0]*w1 + n2[0]*w2)/(w1 + w2);
		// normal_sphere->points[shortest_dist_index].y = (n1[1]*w1 + n2[1]*w2)/(w1 + w2);
		// normal_sphere->points[shortest_dist_index].z = (n1[2]*w1 + n2[2]*w2)/(w1 + w2);
		
		features[shortest_dist_index].num_refpoints += features[features[shortest_dist_index].neighbor_index].num_refpoints;
		features[shortest_dist_index].fitting_error = (features[shortest_dist_index].fitting_error + features[features[shortest_dist_index].neighbor_index].fitting_error)/2.0;
		features[shortest_dist_index].ang_from_g_est = angle_between_vectors(merged_n, g_est);
		features[shortest_dist_index].num_groups += features[features[shortest_dist_index].neighbor_index].num_groups;
		
		// const std::vector<float> features_parameters = {1.0, 1.0, 10.0, 1.0};
		// features[shortest_dist_index].weight = 
		//	features_parameters[0]*features[shortest_dist_index].num_refpoints
		//	*features_parameters[1]*(1/features[shortest_dist_index].fitting_error)
		//	*features_parameters[2]*(1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_est))
		//	*features_parameters[3]*features[shortest_dist_index].num_groups;
		
		// features[shortest_dist_index].weight = 
		// 	PARAMETERS_FOR_FEATURES[0]*features[shortest_dist_index].num_refpoints
		// 	*PARAMETERS_FOR_FEATURES[1]*(1/features[shortest_dist_index].fitting_error)
		// 	*PARAMETERS_FOR_FEATURES[2]*(1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_est))
		// 	*PARAMETERS_FOR_FEATURES[3]*features[shortest_dist_index].num_groups;

		features[shortest_dist_index].weight = 
			FACTOR_1*features[shortest_dist_index].num_refpoints
			+FACTOR_2*(1/features[shortest_dist_index].fitting_error)
			+FACTOR_3*(1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_est))
			+FACTOR_4*features[shortest_dist_index].num_groups;
		
		std::cout << "features[shortest_dist_index].num_refpoints = " << features[shortest_dist_index].num_refpoints << std::endl;
		std::cout << "(1/features[shortest_dist_index].fitting_error) = " << (1/features[shortest_dist_index].fitting_error) << std::endl;
		std::cout << "(1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_est)) = " << (1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_est)) << std::endl;
		std::cout << "features[shortest_dist_index].num_groups = " << features[shortest_dist_index].num_groups << std::endl;


		// num_members[shortest_dist_index] += num_members[pointIdxNKNSearch[shortest_dist_index]];
		
		// std::cout << "erase " << features[shortest_dist_index].neighbor_index << std::endl;
		normal_sphere->points.erase(normal_sphere->points.begin() + features[shortest_dist_index].neighbor_index);
		features.erase(features.begin() + features[shortest_dist_index].neighbor_index);
		// for(int j=0;j<features.size();j++)	std::cout << j << " : features[j].neighbor_index = " << features[j].neighbor_index << std::endl;
		// fitting_errors.erase(fitting_errors.begin()+pointIdxNKNSearch[shortest_dist_index]);
		// num_members.erase(num_members.begin()+pointIdxNKNSearch[shortest_dist_index]);
	}
	for(size_t i=0;i<features.size();i++){
		if(features[i].num_groups<2){
			std::cout << ">> no merge, then erace " << i << std::endl;
			normal_sphere->points.erase(normal_sphere->points.begin() + i);
			features.erase(features.begin() + i);
		}
	}
}

void normals_to_points(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr points)
{
	for(size_t i=0;i<normals->points.size();i++){
		pcl::PointXYZ tmp_point;
		tmp_point.x = normals->points[i].normal_x;
		tmp_point.y = normals->points[i].normal_y;
		tmp_point.z = normals->points[i].normal_z;
		points->points.push_back(tmp_point);
	}
}

void points_to_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr points, pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals)
{
	for(size_t i=0;i<points->points.size();i++){
		pcl::PointXYZINormal tmp_normal;
		tmp_normal.x = 0.0;
		tmp_normal.y = 0.0;
		tmp_normal.z = 1.0;
		tmp_normal.normal_x = points->points[i].x;
		tmp_normal.normal_y = points->points[i].y;
		tmp_normal.normal_z = points->points[i].z;
		normals->points.push_back(tmp_normal);
	}
}

	int main(int argc, char** argv)
	{
	ros::init(argc, argv, "pc_normal");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	
	/*---getParam---*/
	local_nh.getParam("LOOP_RATE", LOOP_RATE);
	local_nh.getParam("SEARCH_RADIUS", SEARCH_RADIUS);
	local_nh.getParam("RANDOM_STEP_MAX", RANDOM_STEP_MAX);
	local_nh.getParam("THRESHOLD_ANGLE_FROM_G", THRESHOLD_ANGLE_FROM_G);
	local_nh.getParam("THRESHOLD_SUM_SQUARE_ERRORS", THRESHOLD_SUM_SQUARE_ERRORS);
	local_nh.getParam("MIN_DISTANCE_BETWEEN_NORMSLS", MIN_DISTANCE_BETWEEN_NORMSLS);
	// local_nh.getParam("PARAMETERS_FOR_FEATURES", PARAMETERS_FOR_FEATURES);
	local_nh.getParam("FACTOR_1", FACTOR_1);
	local_nh.getParam("FACTOR_2", FACTOR_2);
	local_nh.getParam("FACTOR_3", FACTOR_3);
	local_nh.getParam("FACTOR_4", FACTOR_4);

	/*sub & pub*/
	ros::Subscriber cloud_sub = nh.subscribe("/cloud/lcl", 1, cloud_callback);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/cloud",1);
	ros::Publisher normals_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/normals",1);

	// const int num_points = 10000;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::io::loadPCDFile ("/home/amsl/ros_catkin_ws/src/beginner_tutorials/map_0.pcd", *cloud);
	// int num_points = cloud->points.size();
	// cloud->points.resize(num_points);
	// pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals (new pcl::PointCloud<pcl::PointXYZINormal>);
	// normals->points.resize(cloud->points.size());
	// randomize_cloud(cloud, num_points);
	// print_cloud(cloud, cloud->points.size());
	// pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	// normals->points.resize(cloud->points.size());
	// estimate_normal(cloud, normals);

	// Eigen::Vector4f plane_parameters;
	// std::vector<float> fitting_errors;
	// plane_fitting(cloud, normals, planecloud, fitting_errors);
	// plane_fitting(normals, fitting_errors);
	// randomize_normals(normals, 100);
	
	// pcl::PointCloud<pcl::PointXYZ>::Ptr g_point (new pcl::PointCloud<pcl::PointXYZ>);
	// g_point->points.resize(1);
	// pcl::PointCloud<pcl::Normal>::Ptr g_vector (new pcl::PointCloud<pcl::Normal>);
	// g_vector->points.resize(1);
	// estimate_g_vector(normals, g_point, g_vector);

	pcl::visualization::PCLVisualizer viewer("Test Point Cloud Viewer");
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

	// viewer.addPointCloud(cloud, "cloud");
	// std::cout << "TEST" << std::endl;
	ros::Rate loop_rate(10);
	while(ros::ok()){
		viewer.spinOnce();
		// viewer.removePointCloud("cloud");
		// viewer.addPointCloud(cloud, "cloud");
		

		if(!cloud->points.empty()){
			/*-----clouds------*/
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr main_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr g_vector (new pcl::PointCloud<pcl::PointXYZINormal>);
			g_vector->points.resize(1);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_before_clustering (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_after_clustering (new pcl::PointCloud<pcl::PointXYZINormal>);


			/*-----compute-----*/
			// std::vector<float> fitting_errors;
			// std::vector<int> num_refpoints;
			std::vector<FEATURES> features;
			// plane_fitting(normals, fitting_errors, num_refpoints);
			plane_fitting(normals, features);
			std::cout << "marker" << std::endl;
			normals_to_points(normals, normal_sphere);
			points_to_normals(normal_sphere, normals_before_clustering);
			clustering(normals, features, normal_sphere);
			points_to_normals(normal_sphere, normals_after_clustering);
			if(estimate_g_vector(normal_sphere, g_vector)==false){
				std::cout << "FALSE" << std::endl;
				// exit(1);
			}
			// if(g_vector->points[0].normal_z>-0.6){
			// 	std::cout << "WEIRED ESTIMATION" << std::endl;
			// 	exit(1);
			// }
			
			/*-----publish-----*/
			sensor_msgs::PointCloud2 roscloud_out;
			pcl::toROSMsg(*cloud, roscloud_out);
			roscloud_out.header.frame_id = "/centerlaser";
			// roscloud_out.header.stamp = tm;
			cloud_pub.publish(roscloud_out);

			sensor_msgs::PointCloud2 rosnormals_out;
			pcl::toROSMsg(*normals, rosnormals_out);
			rosnormals_out.header.frame_id = "/centerlaser";
			// rosnormals_out.header.stamp = tm;
			normals_pub.publish(rosnormals_out);


			/*-----pcl viewer-----*/
			viewer.removePointCloud("cloud");
			viewer.addPointCloud(cloud, "cloud");
			
			viewer.removePointCloud("normals");
			viewer.addPointCloudNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>(normals, normals, 1, 0.2, "normals");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");
			
			viewer.removePointCloud("g");
			// viewer.addPointCloudNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>(g_vector, g_vector, 1, 0.5, "g");
			viewer.addPointCloudNormals<pcl::PointXYZINormal>(g_vector, 1, 0.2, "g");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "g");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "g");
			
			viewer.removePointCloud("normals_before_clustering");
			viewer.addPointCloudNormals<pcl::PointXYZINormal>(normals_before_clustering, 1, 0.3, "normals_before_clustering");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normals_before_clustering");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normals_before_clustering");
			
			viewer.removePointCloud("normals_after_clustering");
			viewer.addPointCloudNormals<pcl::PointXYZINormal>(normals_after_clustering, 1, 0.2, "normals_after_clustering");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "normals_after_clustering");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "normals_after_clustering");

			// if(g_vector->points[0].normal_z>-0.0){
			// 	std::cout << "WEIRED ESTIMATION" << std::endl;
			// 	while(1){}
			// }

		}
		// viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		ros::spinOnce();
		loop_rate.sleep();
	}
}
