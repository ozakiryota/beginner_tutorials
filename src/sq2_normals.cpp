/*
 *	pc_normal.cpp
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

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<float> g_local;
bool g_local_is_available = false;

struct	FEATURES{
	size_t num_refpoints;
	float fitting_error;
	float ang_from_g_local;
	float weight;
	int neighbor_index;
	int num_groups;
};

/*-----getParam-----*/
float LOOP_RATE;
float SEARCH_RADIUS;
int RANDOM_STEP_MAX;
int THRESHOLD_REF_POINTS;
float THRESHOLD_ANGLE_FROM_G;
float THRESHOLD_SUM_SQUARE_ERRORS;
float MIN_DISTANCE_BETWEEN_NORMSLS;
float MIN_NUM_GROUPS;
float FACTOR_1;
float FACTOR_2;
float FACTOR_3;
float FACTOR_4;
std::string EST_POSEMSG_NAME;

// void print_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int n)
// {
// 	for(int i=0;i<n;i++)	std::cout << i+1 << ": " << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << std::endl;
// }

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

void create_another_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere)
{
	std::cout << "---CREATE ANOTHER NORMAL---" << std::endl;
	float another_n_x = normal_sphere->points[0].y*g_local[2]
						-normal_sphere->points[0].z*g_local[1];
	float another_n_y = normal_sphere->points[0].z*g_local[0]
						-normal_sphere->points[0].x*g_local[2];
	float another_n_z = normal_sphere->points[0].x*g_local[1]
						-normal_sphere->points[0].y*g_local[0];
	float norm = sqrt(another_n_x*another_n_x + another_n_y*another_n_y + another_n_z*another_n_z);
	pcl::PointXYZ another_n;
	another_n.x = another_n_x/norm;
	another_n.y = another_n_y/norm;
	another_n.z = another_n_z/norm;
	
	normal_sphere->points.push_back(another_n);
}

bool estimate_g_vector(pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere, pcl::PointCloud<pcl::PointNormal>::Ptr g_vector)
{
	std::cout << "-----ESTIMATE G_VECTOR-----" << std::endl;
	// if(normal_sphere->points.size()<2){
	// 	std::cout << "Failed because normal_sphere has less than 2 normals" << std::endl;
	// 	return false;
	// }
	if(normal_sphere->points.size()<1){
		std::cout << "Failed because normal_sphere has no normal" << std::endl;	
		return false;
	}

	if(normal_sphere->points.size()==1){
		std::cout << "normal_sphere has just one normal" << std::endl;
		create_another_normal(normal_sphere);
	}
	else	std::cout << "normal_sphere has more than 2 normals" << std::endl;

	/*the center of Gauss Sphere*/
	g_vector->points[0].x = 0;
	g_vector->points[0].y = 0;
	g_vector->points[0].z = 1.0;

	std::cout << "normal_sphere->points.size() = " << normal_sphere->points.size() << std::endl;
	if(normal_sphere->points.size()==2){
		/*cross product*/
		float g_x = normal_sphere->points[0].y*normal_sphere->points[1].z - normal_sphere->points[0].z*normal_sphere->points[1].y;
		float g_y = normal_sphere->points[0].z*normal_sphere->points[1].x - normal_sphere->points[0].x*normal_sphere->points[1].z;
		float g_z = normal_sphere->points[0].x*normal_sphere->points[1].y - normal_sphere->points[0].y*normal_sphere->points[1].x;

		float norm = sqrt(g_x*g_x + g_y*g_y + g_z*g_z);
		
		g_vector->points[0].normal_x = g_x/norm;
		g_vector->points[0].normal_y = g_y/norm;
		g_vector->points[0].normal_z = g_z/norm;

		std::cout << "g_vector->points[0] = " << g_vector->points[0] << std::endl;
		flipNormalTowardsViewpoint(g_vector->points[0], 0.0, 0.0, 0.5, g_vector->points[0].normal_x, g_vector->points[0].normal_y, g_vector->points[0].normal_z);
		return true;
	}

	Eigen::Vector4f g_parameters;
	float curvature; 
	pcl::computePointNormal(*normal_sphere, g_parameters, curvature);
	flipNormalTowardsViewpoint(g_vector->points[0], 0.0, 0.0, 0.5, g_parameters);
	g_vector->points[0].normal_x = g_parameters[0];
	g_vector->points[0].normal_y = g_parameters[1];
	g_vector->points[0].normal_z = g_parameters[2];
	std::cout << "g_vector->points[0] = " << g_vector->points[0] << std::endl;
	return true;
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
	if(fabs(acosf(dot_product/(v1_norm*v2_norm)))>M_PI){
		std::cout << "fabs(acosf(dot_product/(v1_norm*v2_norm)))>M_PI" << std::endl;
		exit(1);
	}
	return fabs(acosf(dot_product/(v1_norm*v2_norm)));
}

void clustering(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, std::vector<FEATURES> features, pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere)
{
	std::cout << "-----CLUSTERING-----" << std::endl;
	std::cout << "normals->points.size() = " << normals->points.size() << std::endl;
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	int k = 2;
	pcl::PointXYZ searchpoint;
	
	while(ros::ok()){
		if(normal_sphere->points.size()<k){
			std::cout << "= END MERGE =" << std::endl;
			break;
		}

		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);
		kdtree.setInputCloud(normal_sphere);
		float shortest_dist_index;	//A pair of points[shortest_dist_index] and points[features[neighbor_index]] is the closest
		float shortest_distance;
		for(int i=0;i<normal_sphere->points.size();i++){
			searchpoint = normal_sphere->points[i];
			if(kdtree.nearestKSearch(searchpoint, k, pointIdxNKNSearch, pointNKNSquaredDistance)<=0){
				std::cout << "error" << std::endl;
				// exit(1);
				break;
			}
			
			features[i].neighbor_index = pointIdxNKNSearch[1];
			
			std::cout << i << ": normal_sphere->points[i] = " << normal_sphere->points[i] << std::endl;
			std::cout << "   features[i].num_refpoints = " << features[i].num_refpoints << std::endl;
			// std::cout << "   features[i].fitting_error = " << features[i].fitting_error << std::endl;
			std::cout << "   1/features[i].fitting_error = " << 1/features[i].fitting_error << std::endl;
			// std::cout << "   features[i].ang_from_g_est = " << features[i].ang_from_g_est << std::endl;
			std::cout << "   1/fabs(M_PI/2.0 - features[i].ang_from_g_local) = " << 1/fabs(M_PI/2.0 - features[i].ang_from_g_local) << std::endl;
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
		if(shortest_distance>MIN_DISTANCE_BETWEEN_NORMSLS){
			std::cout << "= END MERGE =" << std::endl;
			break;
		}
		
		std::cout << "MERGE " << shortest_dist_index << " & " << features[shortest_dist_index].neighbor_index << std::endl;
		std::vector<float> n1 = {
			normal_sphere->points[shortest_dist_index].x,
			normal_sphere->points[shortest_dist_index].y,
			normal_sphere->points[shortest_dist_index].z
		};
		float w1 = features[shortest_dist_index].weight;
		std::vector<float> n2 = {
			normal_sphere->points[features[shortest_dist_index].neighbor_index].x,
			normal_sphere->points[features[shortest_dist_index].neighbor_index].y,
			normal_sphere->points[features[shortest_dist_index].neighbor_index].z
		};
		float w2 = features[features[shortest_dist_index].neighbor_index].weight;
		
		std::cout << "w1 = " << w1 << std::endl;
		std::cout << "w2 = " << w2 << std::endl;
		std::cout << "w1/(w1 + w2) = " << w1/(w1 + w2) << std::endl;

		std::vector<float> merged_n = merge_vectors(n1, n2, w1, w2);
		normal_sphere->points[shortest_dist_index].x = merged_n[0];
		normal_sphere->points[shortest_dist_index].y = merged_n[1];
		normal_sphere->points[shortest_dist_index].z = merged_n[2];
		
		features[shortest_dist_index].num_refpoints += features[features[shortest_dist_index].neighbor_index].num_refpoints;
		features[shortest_dist_index].fitting_error = (features[shortest_dist_index].fitting_error + features[features[shortest_dist_index].neighbor_index].fitting_error)/2.0;
		features[shortest_dist_index].ang_from_g_local = angle_between_vectors(merged_n, g_local);
		features[shortest_dist_index].num_groups += features[features[shortest_dist_index].neighbor_index].num_groups;
		features[shortest_dist_index].weight = 
			FACTOR_1*features[shortest_dist_index].num_refpoints
			+ FACTOR_2*(1/features[shortest_dist_index].fitting_error)
			+ FACTOR_3*(1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_local))
			+ FACTOR_4*features[shortest_dist_index].num_groups;
		
		std::cout << "features[shortest_dist_index].num_refpoints = " << features[shortest_dist_index].num_refpoints << std::endl;
		std::cout << "(1/features[shortest_dist_index].fitting_error) = " << (1/features[shortest_dist_index].fitting_error) << std::endl;
		std::cout << "(1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_local)) = " << (1/fabs(M_PI/2.0 - features[shortest_dist_index].ang_from_g_local)) << std::endl;
		std::cout << "features[shortest_dist_index].num_groups = " << features[shortest_dist_index].num_groups << std::endl;
	
		/*erase*/
		normal_sphere->points.erase(normal_sphere->points.begin() + features[shortest_dist_index].neighbor_index);
		features.erase(features.begin() + features[shortest_dist_index].neighbor_index);
	}

	for(size_t i=0;i<features.size();i++){
		if(features[i].num_groups<MIN_NUM_GROUPS){
			std::cout << ">> not enough number of merge, then erase " << i << std::endl;
			normal_sphere->points.erase(normal_sphere->points.begin() + i);
			features.erase(features.begin() + i);
		}
	}
}

std::vector<int> kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ searchpoint)	//return neighbors' index
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if(kdtree.radiusSearch(searchpoint, SEARCH_RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance)<=0)	std::cout << "kdtree error" << std::endl;
	return pointIdxRadiusSearch; 
}

void plane_fitting(pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals, pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_flipped, std::vector<FEATURES>& features)
{
	std::cout << "-----PLANE FITTING-----" <<  std::endl;
	float curvature;
	std::random_device rnd;
	std::mt19937 mt(rnd());
	std::uniform_int_distribution<> rand_step(1, RANDOM_STEP_MAX);
	int i = -1;
	while(ros::ok()){
		i += rand_step(mt);
		if(i>=cloud->points.size())	break;
		
		if(cloud->points[i].z>0.0 && cloud->points[i].z<0.35){
			std::cout << ">> cloud->points[i].z is out of the range<, then skip" << std::endl;
			continue;
		}
		
		// const double threshold_height = 0.2;
		// if(cloud->points[i].z<threshold_height){
		// 	std::cout << ">> cloud->points[i].z< "<< threshold_height << ", then skip" << std::endl;
		// 	continue;
		// }

		pcl::PointXYZ searchpoint;
		searchpoint.x = cloud->points[i].x;
		searchpoint.y = cloud->points[i].y;
		searchpoint.z = cloud->points[i].z;

		// std::cout << "start finding kdtree" <<  std::endl;
		std::vector<int> indices = kdtree_search(cloud, searchpoint);
		std::cout << "indices.size() = " << indices.size() << std::endl;
		if(indices.size()<THRESHOLD_REF_POINTS){
			std::cout << ">> indices.size()< "<< THRESHOLD_REF_POINTS << ", then skip" << std::endl;
			continue;
		}

		Eigen::Vector4f plane_parameters;
		pcl::computePointNormal(*cloud, indices, plane_parameters, curvature);
		std::vector<float> tmp_vector = {plane_parameters[0], plane_parameters[1], plane_parameters[2]};
		float tmp_ang_from_g_local = angle_between_vectors(tmp_vector, g_local);
		if(tmp_ang_from_g_local<THRESHOLD_ANGLE_FROM_G/180.0*M_PI || tmp_ang_from_g_local>(M_PI-THRESHOLD_ANGLE_FROM_G/180.0*M_PI)){
			std::cout << ">> tmp_ang_from_g_local is too small or large, then skip" << std::endl;
			continue;
		}
		std::cout << "OK" << std::endl;
		
		// std::cout << "start caluculating square_error" <<  std::endl;
		float sum_square_error = 0.0;
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
		// std::cout << "sum_square_error = " << sum_square_error << std::endl;
		// std::cout << "curvature = " << plane_parameters[3] << std::endl;
		// const float sum_square_error_threshold = 0.1;
		
		if(sum_square_error>THRESHOLD_SUM_SQUARE_ERRORS)	std::cout << ">> sum_square_error is too large, then skip" << std::endl;
		else{
			pcl::PointXYZINormal tmp_normal;
			tmp_normal.x = cloud->points[i].x;
			tmp_normal.y = cloud->points[i].y;
			tmp_normal.z = cloud->points[i].z;
			// flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 1.0, plane_parameters);
			tmp_normal.normal_x = plane_parameters[0];
			tmp_normal.normal_y = plane_parameters[1];
			tmp_normal.normal_z = plane_parameters[2];
			normals->points.push_back(tmp_normal);
			
			features.push_back({
				indices.size(),
				sum_square_error,
				tmp_ang_from_g_local,
				FACTOR_1*indices.size()
				+ FACTOR_2*(1/sum_square_error)
				+ FACTOR_3*(1/fabs(M_PI/2.0 - tmp_ang_from_g_local)),
				i,
				1});

			/*just for viewing*/
			flipNormalTowardsViewpoint(tmp_normal, 0.0, 0.0, 1.0, plane_parameters);
			tmp_normal.normal_x = plane_parameters[0];
			tmp_normal.normal_y = plane_parameters[1];
			tmp_normal.normal_z = plane_parameters[2];
			normals_flipped->points.push_back(tmp_normal);
		}
	}
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	std::cout << "-----CLOUD CALLBACK-----" << std::endl;
	// pcl::fromROSMsg(*msg, *cloud);
	
	if(false){
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *tmp);
		for(size_t i=0;i<tmp->points.size();i++){
			if(tmp->points[i].z<0.0 || tmp->points[i].z>0.35)	cloud->points.push_back(tmp->points[i]);
		}
	}
	else	pcl::fromROSMsg(*msg, *cloud);
}

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

void callback_pose(const geometry_msgs::PoseConstPtr& msg)
{
	Eigen::MatrixXd G_global(3, 1);
	G_global <<	0.0,
			 	0.0,
				-9.80665;
	Eigen::MatrixXd G_local(3, 1);
	G_local = frame_rotation(msg->orientation, G_global, true);
	g_local = {G_local(0, 0), G_local(1, 0), G_local(2, 0)};

	// std::cout << "G_local = " << std::endl << G_local << std::endl;

	g_local_is_available = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sq2_normals");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");
	
	/*---getParam---*/
	local_nh.getParam("LOOP_RATE", LOOP_RATE);
	local_nh.getParam("SEARCH_RADIUS", SEARCH_RADIUS);
	local_nh.getParam("RANDOM_STEP_MAX", RANDOM_STEP_MAX);
	local_nh.getParam("THRESHOLD_REF_POINTS", THRESHOLD_REF_POINTS);
	local_nh.getParam("THRESHOLD_ANGLE_FROM_G", THRESHOLD_ANGLE_FROM_G);
	local_nh.getParam("THRESHOLD_SUM_SQUARE_ERRORS", THRESHOLD_SUM_SQUARE_ERRORS);
	local_nh.getParam("MIN_DISTANCE_BETWEEN_NORMSLS", MIN_DISTANCE_BETWEEN_NORMSLS);
	local_nh.getParam("MIN_NUM_GROUPS", MIN_NUM_GROUPS);
	// local_nh.getParam("PARAMETERS_FOR_FEATURES", PARAMETERS_FOR_FEATURES);
	local_nh.getParam("FACTOR_1", FACTOR_1);
	local_nh.getParam("FACTOR_2", FACTOR_2);
	local_nh.getParam("FACTOR_3", FACTOR_3);
	local_nh.getParam("FACTOR_4", FACTOR_4);
	local_nh.getParam("EST_POSEMSG_NAME", EST_POSEMSG_NAME);

	/*sub & pub*/
	ros::Subscriber sub_pose = nh.subscribe("/pose_doubleekf", 1, callback_pose);
	ros::Subscriber cloud_sub = nh.subscribe("/cloud/lcl", 1, cloud_callback);
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/cloud",1);
	ros::Publisher normals_pub = nh.advertise<sensor_msgs::PointCloud2>("/test/normals",1);
	ros::Publisher g_pub = nh.advertise<sensor_msgs::PointCloud2>("/g_usingwalls",1);

	pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.5, "axis");

	// ros::Rate loop_rate(LOOP_RATE);
	while(ros::ok()){
		if(!cloud->points.empty()&&g_local_is_available){
			/*-----clouds------*/
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_flipped (new pcl::PointCloud<pcl::PointXYZINormal>);	//just for viewing
			pcl::PointCloud<pcl::PointXYZ>::Ptr normal_sphere (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr main_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::PointCloud<pcl::PointNormal>::Ptr g_vector (new pcl::PointCloud<pcl::PointNormal>);
			g_vector->points.resize(1);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_before_clustering (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_after_clustering (new pcl::PointCloud<pcl::PointXYZINormal>);
			
			pcl::PointCloud<pcl::PointNormal>::Ptr g_last (new pcl::PointCloud<pcl::PointNormal>);
			g_last->points.resize(1);
			g_last->points[0].x = 0;
			g_last->points[0].y = 0;
			g_last->points[0].z = 1.0;
			g_last->points[0].normal_x = g_local[0];
			g_last->points[0].normal_y = g_local[1];
			g_last->points[0].normal_z = g_local[2];


			/*-----compute-----*/
			std::vector<FEATURES> features;
			plane_fitting(normals, normals_flipped, features);
			normals_to_points(normals, normal_sphere);
			points_to_normals(normal_sphere, normals_before_clustering);
			clustering(normals, features, normal_sphere);
			points_to_normals(normal_sphere, normals_after_clustering);
			bool success_estimating = estimate_g_vector(normal_sphere, g_vector);
			if(success_estimating==false){
				std::cout << "FALSE" << std::endl;
				// exit(1);
			}
			
			/*-----publish-----*/
			sensor_msgs::PointCloud2 roscloud_out;
			pcl::toROSMsg(*cloud, roscloud_out);
			// roscloud_out.header.frame_id = "/centerlaser";
			roscloud_out.header.frame_id = "/odom3d_";
			// roscloud_out.header.stamp = tm;
			cloud_pub.publish(roscloud_out);

			sensor_msgs::PointCloud2 rosnormals_out;
			pcl::toROSMsg(*normals, rosnormals_out);
			// rosnormals_out.header.frame_id = "/centerlaser";
			rosnormals_out.header.frame_id = "/odom3d_";
			// rosnormals_out.header.stamp = tm;
			normals_pub.publish(rosnormals_out);
			
			if(success_estimating==true){
				std::cout << "success" << std::endl;
				sensor_msgs::PointCloud2 g_out;
				pcl::toROSMsg(*g_vector, g_out);
				// g_out.header.frame_id = "/centerlaser";
				g_out.header.frame_id = "/odom3d_";
				g_pub.publish(g_out);
			}

			/*-----pcl viewer-----*/
			viewer.removePointCloud("cloud");
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color (cloud, 0, 0, 0);
			viewer.addPointCloud(cloud, cloud_color , "cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

			viewer.removePointCloud("normals");
			viewer.addPointCloudNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>(normals, normals_flipped, 1, 0.2, "normals");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals");
			
			viewer.removePointCloud("g");
			viewer.addPointCloudNormals<pcl::PointNormal>(g_vector, 1, 0.2, "g");
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
			
			viewer.removePointCloud("g_last");
			viewer.addPointCloudNormals<pcl::PointNormal>(g_last, 1, 0.03, "g_last");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.5, 0.5, "g_last");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "g_last");
		}
		ros::spinOnce();
		viewer.spinOnce();
		// loop_rate.sleep();
	}
}
