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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/NavSatFix.h>

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher	pub_cloud;

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	cloud_in->header.frame_id = "rear_lidar";

	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_cloud.publish(cloud_to_pub);
}

void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rear_transform");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu = nh.subscribe("/left/velodyne_points",2,pointCloudCallback);
	ros::Subscriber sub_gps = nh.subscribe("/rtk/fix", 10,GNSSCallback);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("rear_transformed_points",2);

	ros::spin();
	return 0;
}
