#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include "beginner_tutorials/PointWithRange.h"
#include "beginner_tutorials/PointWithRangeArray.h"

class EuclideanClustering{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*publish*/
		ros::Publisher pub_originl;
		/*pcl objects*/
		pcl::visualization::PCLVisualizer viewer {"Euclidian Clustering"};
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		/*original message*/
		beginner_tutorials::PointWithRangeArray point_with_range_array;
		/*parameters*/
		double cluster_tolerance;
		int min_cluster_size;
	public:
		EuclideanClustering();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Clustering(void);
		void Visualization(void);
		void Publication(void);
};

EuclideanClustering::EuclideanClustering()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &EuclideanClustering::CallbackPC, this);
	pub_originl = nh.advertise<beginner_tutorials::PointWithRangeArray>("/point_with_range", 1);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);

	nhPrivate.param("cluster_tolerance", cluster_tolerance, 0.1);
	nhPrivate.param("min_cluster_size", min_cluster_size, 100);
	std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
	std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
}

void EuclideanClustering::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "==========" << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	clusters.clear();
	point_with_range_array.points.clear();
	Clustering();
	Visualization();
	Publication();
}

void EuclideanClustering::Clustering(void)
{
	double time_start = ros::Time::now().toSec();

	/*clustering*/
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
	ece.setClusterTolerance(cluster_tolerance);
	ece.setMinClusterSize(min_cluster_size);
	ece.setMaxClusterSize(cloud->points.size());
	ece.setSearchMethod(tree);
	ece.setInputCloud(cloud);
	ece.extract(cluster_indices);

	std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	/*dividing*/
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud);
	ei.setNegative(false);
	for(size_t i=0;i<cluster_indices.size();i++){
		/*get min-max*/
		Eigen::Vector4f Min;
		Eigen::Vector4f Max;		
		getMinMax3D(*cloud, cluster_indices[i], Min, Max);
		std::cout << i << ": Min  = " << std::endl << Min << std::endl;
		std::cout << i << ": Max  = " << std::endl << Max << std::endl;
		/*extract*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		beginner_tutorials::PointWithRange tmp_point_with_range;
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		ei.setIndices(tmp_clustered_indices);
		ei.filter(*tmp_clustered_points);
		/*compute centroid*/
		Eigen::Vector4f xyz_centroid;
		pcl::compute3DCentroid(*tmp_clustered_points, xyz_centroid);
		/*input*/
		clusters.push_back(tmp_clustered_points);
		tmp_point_with_range.x = xyz_centroid[0];
		tmp_point_with_range.y = xyz_centroid[1];
		tmp_point_with_range.z = xyz_centroid[2];
		tmp_point_with_range.min_x = Min[0];
		tmp_point_with_range.min_y = Min[1];
		tmp_point_with_range.min_z = Min[2];
		tmp_point_with_range.max_x = Max[0];
		tmp_point_with_range.max_y = Max[1];
		tmp_point_with_range.max_z = Max[2];
		point_with_range_array.points.push_back(tmp_point_with_range);
	}

	std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

void EuclideanClustering::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*cloud*/
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	/*clusters*/
	double rgb[3] = {};
	const int channel = 3;
	const double step = ceil(pow(clusters.size()+2, 1.0/(double)channel));	//exept (000),(111)
	const double max = 1.0;
	for(size_t i=0;i<clusters.size();i++){
		std::string name = "cluster_" + std::to_string(i);
		rgb[0] += 1/step;
		for(int j=0;j<channel-1;j++){
			if(rgb[j]>max){
				rgb[j] -= max + 1/step;
				rgb[j+1] += 1/step;
			}
		}
		viewer.addPointCloud(clusters[i], name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
	}
	
	viewer.spinOnce();
}

void EuclideanClustering::Publication(void)
{
	pub_originl.publish(point_with_range_array);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering");
	
	EuclideanClustering euclidean_clustering;

	ros::spin();
}
