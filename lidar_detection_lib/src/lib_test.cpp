#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <lidar_detection/common_headers.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_detection_test");
	return 0;
}
