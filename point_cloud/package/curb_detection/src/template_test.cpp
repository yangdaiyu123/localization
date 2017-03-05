#include "ros/ros.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

#include <pcl_ros/point_cloud.h>
#include <point_types.h>
#include <cloud_operation.h>
#include <point_xyzo.h>

using namespace cloud_operation;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_template");

	pcl::PointXYZ pt0;
	pt0.x = 1; pt0.y = 1; pt0.z = 1;

	pcl::PointXYZI pt1;
	pt1.x = 2; pt1.y = 2; pt1.z = 2; pt1.intensity = 100;

	PointXYZO pt2;
	pt2.x = 3; pt2.y = 3; pt2.z = 3; pt2.orientation = 1.5;

	pcl::PointXYZRGB pt3;
	pt3.x = 4; pt3.y = 4; pt3.z = 4; pt3.rgb = 0;

	filter::test_template(pt0);
	filter::test_template(pt1);
	filter::test_template(pt2);
	filter::test_template(pt3);

	return 0;
}
