#include <ros/ros.h>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointNormal>::Ptr normal (new pcl::PointCloud<pcl::PointNormal>);
pcl::visualization::PCLVisualizer viewer("test_least_squares_method");

void input_cloud(void)
{
	pcl::PointXYZ tmp_point;
	/*point1*/
	tmp_point.x = 1.0;
	tmp_point.y = 0.0;
	tmp_point.z = 0.0;
	cloud->points.push_back(tmp_point);
	/*point2*/
	tmp_point.x = 0.0;
	tmp_point.y = 1.0;
	tmp_point.z = 0.0;
	cloud->points.push_back(tmp_point);
	/*point3*/
	tmp_point.x = 1.0/sqrt(2.0);
	tmp_point.y = 0.0;
	tmp_point.z = 1.0/sqrt(2.0);
	cloud->points.push_back(tmp_point);
}

void compute_normal(void)
{
	normal->points.resize(1);
	normal->points[0].x = 0.0;
	normal->points[0].y = 0.0;
	normal->points[0].z = 0.0;

	Eigen::Vector4f plane_parameters;
	float curvature;
	pcl::computePointNormal(*cloud, plane_parameters, curvature);
	normal->points[0].normal_x = plane_parameters[0];
	normal->points[0].normal_y = plane_parameters[1];
	normal->points[0].normal_z = plane_parameters[2];
	std::cout << normal->points[0] << std::endl;
}

void visualize(void)
{
	viewer.removePointCloud("cloud");
	viewer.addPointCloud(cloud, "cloud");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	
	viewer.removePointCloud("normal");
	viewer.addPointCloudNormals<pcl::PointNormal>(normal, 1, 0.5, "normal");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "normal");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "normal");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_least_squares_method");
	ros::NodeHandle nh;

	input_cloud();
	compute_normal();
	visualize();
	viewer.addCoordinateSystem(0.5, "axis");
	
	ros::Rate loop_rate(1);
	while(ros::ok()){
		viewer.spinOnce();
		loop_rate.sleep();
	}
}
