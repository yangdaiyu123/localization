#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <sstream>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_demo");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registration (new pcl::PointCloud<pcl::PointXYZ>);


	// check arguments
	if(argc != 3) {
		std::cout << "ERROR: the number of arguments is illegal!" << std::endl;
		return -1;
	}

	// load pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source)==-1) {
		std::cout << "load source failed!" << std::endl;
		return -1;
	}
	std::cout << "source loaded!" << std::endl;

	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_target)==-1) {
		std::cout << "load target failed!" << std::endl;
		return -1;
	}
	std::cout << "target loaded!" << std::endl;

	// ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_source);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_target);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(1500);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.1);
	icp.setMaximumIterations(300);
	icp.align(*cloud_source_registration);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	std::cout << transformation << std::endl;

	// display
	pcl::visualization::PCLVisualizer p;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_r_h(cloud_source_registration, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 0, 0, 255);
	p.addPointCloud(cloud_source_registration, src_r_h,"source_r");
	p.addPointCloud(cloud_target, tgt_h, "target");
	p.addPointCloud(cloud_source, src_h, "source");
	p.spin();
	return 0;


}
