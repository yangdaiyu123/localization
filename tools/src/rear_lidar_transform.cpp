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

#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <tools/lidar_calibrationConfig.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher	pub_front;
ros::Publisher	pub_rear;

double front_pitch = 0;
double rear_pitch = 0.22;
double rear_tx = 0;
double rear_ty = 0;
double rear_tz = 0;

void rearPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << rear_tx, rear_ty, rear_tz;
	transform.rotate(Eigen::AngleAxisf(-M_PI/2.0, Eigen::Vector3f::UnitZ()));
	transform.rotate(Eigen::AngleAxisf(rear_pitch, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);

	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_rear.publish(cloud_to_pub);
}

void frontPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0.0, 0.0, 0.0;
	transform.rotate(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
	transform.rotate(Eigen::AngleAxisf(front_pitch, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);

	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_front.publish(cloud_to_pub);
}

void configCallback(localization::rearConfig &config, uint32_t level)
{
	front_pitch = config.front_pitch;
	rear_pitch = config.rear_pitch;
	rear_tx = config.rear_tx;
	rear_ty = config.rear_ty;
	rear_tz = config.rear_tz;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_calibration");
	ros::NodeHandle nh;
	ros::Subscriber sub_imu = nh.subscribe("/left/velodyne_points",2,rearPointCloudCallback);
	ros::Subscriber sub_imu = nh.subscribe("velodyne_points",2,frontPointCloudCallback);
	pub_rear = nh.advertise<sensor_msgs::PointCloud2>("rear_points_transformed",2);
	pub_front = nh.advertise<sensor_msgs::PointCloud2>("front_points_transformed",2);

	//---------参数服务相关变量------------
	dynamic_reconfigure::Server<localization::rearConfig> dr_srv;
	dynamic_reconfigure::Server<localization::rearConfig>::CallbackType cb;
	//------配置动态更改参数服务-------
	cb = boost::bind(&configCallback, _1, _2);
	dr_srv.setCallback(cb);

	ros::spin();
	return 0;
}
