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
	ros::init(argc, argv, "show_pcd_transform");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);


	// check arguments
	if(argc != 2) {
		std::cout << "ERROR: the number of arguments is illegal!" << std::endl;
		return -1;
	}

	// load pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source)==-1) {
		std::cout << "load source failed!" << std::endl;
		return -1;
	}
	std::cout << "source loaded!" << std::endl;

	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << 10, 30, 0;
	trans.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_source, *cloud_trans, trans);

	// display
	pcl::visualization::PCLVisualizer p;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src (cloud_source, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_tr (cloud_trans, 0, 255, 0);
	p.addPointCloud(cloud_source, src, "source");
	p.addPointCloud(cloud_trans, src_tr, "transformed");
	p.spin();
	return 0;


}
