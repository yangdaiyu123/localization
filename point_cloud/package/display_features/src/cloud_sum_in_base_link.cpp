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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace message_filters;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher	pub_sum;

double front_pitch = 14.0;//degree
double rear_roll = -22.0;//degree
double rear_pitch = 0;//degree
double rear_yaw = 90.0;//degree
double front_tx = 0;
double front_ty = 0;
double front_tz = 1.9;
double rear_tx = -0.8;
double rear_ty = -0.2;
double rear_tz = 1.7;


void synCurbCallback(const sensor_msgs::PointCloud2::ConstPtr& front_cloud_in,
					 const sensor_msgs::PointCloud2::ConstPtr& rear_cloud_in)
{
	pcl::fromROSMsg(*front_cloud_in, *cloud_front);
	pcl::fromROSMsg(*rear_cloud_in, *cloud_rear);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << front_tx, front_ty, front_tz;
	transform.rotate(Eigen::AngleAxisf(front_pitch*M_PI/180, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_front, *cloud_front, transform);


	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	transform2.translation() << rear_tx, rear_ty, rear_tz;
	transform2.rotate(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
	transform2.rotate(Eigen::AngleAxisf(rear_pitch*M_PI/180, Eigen::Vector3f::UnitY()));
	transform2.rotate(Eigen::AngleAxisf(rear_roll*M_PI/180, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*cloud_rear, *cloud_rear, transform2);


	*cloud_sum = *cloud_front + *cloud_rear;
	cloud_sum->header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_sum, cloud_to_pub);
    pub_sum.publish(cloud_to_pub);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "all_points_in_base_link");
	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_rear_cloud(nh, "/rear/velodyne_points", 2);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_front_cloud(nh, "/velodyne_points", 2);
	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_front_cloud, sub_rear_cloud);
	sync.registerCallback(boost::bind(&synCurbCallback, _1, _2));

	pub_sum = nh.advertise<sensor_msgs::PointCloud2>("cloud_sum_in_base_link",2);

	ros::NodeHandle pnh("~");
	pnh.param("front_pitch", front_pitch,14.0);
	pnh.param("front_tx", front_tx,0.0);
	pnh.param("front_ty", front_ty,0.0);
	pnh.param("front_tz", front_tz,1.9);
	pnh.param("rear_roll", rear_roll,-22.0);
	pnh.param("rear_pitch", rear_pitch,0.0);
	pnh.param("rear_yaw", rear_yaw,90.0);
	pnh.param("rear_tx", rear_tx,-0.8);
	pnh.param("rear_ty", rear_ty,-0.2);
	pnh.param("rear_tz", rear_tz,1.7);

	ros::spin();
	return 0;
}