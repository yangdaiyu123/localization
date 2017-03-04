#ifndef _ICP_LOCALIZER_H_
#define _ICP_LOCALIZER_H_

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

#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <sstream>

#include "cloud_accumulator.h"
#include <pcl_ros/point_cloud.h>
#include "point_xyzo.h"

#include <vector>


class ICPLocalizer
{
public:
	ICPLocalizer();
	void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input);
	void gpsCallback(const sensor_msgs::NavSatFixConstPtr& input);
	void headingCallback(const std_msgs::Float64::ConstPtr& input);
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& input);
	void frontCurbCallback(const OPointCloud::ConstPtr &input);
	void rearCurbCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void signCallback(const sensor_msgs::PointCloud2::ConstPtr& input);




private:
	ros::NodeHandle nh;
	ros::Subscriber sub_pulse;
	ros::Subscriber sub_gps;
	ros::Subscriber sub_heading;
	ros::Subscriber sub_pose;
	ros::Subscriber sub_imu;
	ros::Subscriber sub_front_curb;
	ros::Subscriber sub_rear_curb;
	ros::Subscriber sub_sign;

	ros::Publisher pub_cloud_sum;

	Cloudaccumulator<pcl::PointXYZ> front_accu;

	float front_odom_inc;
	float rear_odom_inc;
	float sign_odom_inc;
	float front_yaw_inc;
	float rear_yaw_inc;
	float sign_yaw_inc;

	Eigen::Affine3f b_to_m;
	pcl::PointCloud<PointXYZO>::Ptr cloud_sum;
	std::vector<int> fifo_size;

	tf::StampedTransform trans_front;
};












#endif
