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
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "point_xyzo.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "point_xyzo_demo");
	ros::NodeHandle nh;

//	pcl::PointCloud<PointXYZO> cloud;
	pcl::PointCloud<PointXYZO>::Ptr cloud (new pcl::PointCloud<PointXYZO>);


	cloud->width    = 1000;
	cloud->height   = 1;
	cloud->is_dense = false;
	cloud->points.resize (cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
		cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud->points[i].orientation = 1.57;
	}


	// display
	pcl::visualization::PCLVisualizer p;
	pcl::visualization::PointCloudColorHandlerCustom<PointXYZO> src (cloud, 0, 0, 255);
	p.addPointCloud(cloud, src, "source");

	PointXYZO pt2,pt3;
	pt2.x = cloud->points[0].x + 2 * cos(cloud->points[0].orientation);
	pt2.y = cloud->points[0].y + 2 * sin(cloud->points[0].orientation);
	pt2.z = cloud->points[0].z;
	pt3.x = cloud->points[0].x + 2 * sin(cloud->points[0].orientation);
	pt3.y = cloud->points[0].y + 2 * cos(cloud->points[0].orientation);
	pt3.z = cloud->points[0].z;
	p.addLine(cloud->points[0], pt2, 255, 0, 0, "line1");
	p.addLine(cloud->points[0], pt3, 255, 0, 0, "line2");

	p.spin();
	return 0;


}
