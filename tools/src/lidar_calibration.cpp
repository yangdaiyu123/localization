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

double front_pitch = 0;//degree
double rear_roll = 0;//degree
double rear_pitch = 0;//degree
double rear_yaw = 0;//degree
double front_tx = 0;
double front_ty = 0;
double front_tz = 0;
double rear_tx = 0;
double rear_ty = 0;
double rear_tz = 0;

void rearPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_in->begin(); it != cloud_in->end(); /**/)
	{
		if ( ((*it).x*(*it).x)+((*it).y*(*it).y)  < 1.5)
			it = cloud_in->erase(it);
		else
			++it;
	}

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << rear_tx, rear_ty, rear_tz;
	transform.rotate(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
	transform.rotate(Eigen::AngleAxisf(rear_pitch*M_PI/180, Eigen::Vector3f::UnitY()));
	transform.rotate(Eigen::AngleAxisf(rear_roll*M_PI/180, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);

	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-15,15);
	pass.filter(*cloud_in);
	
	cloud_in->header.frame_id = "base_link";
	
	for(int i=0; i<1000; i++)
			cout<<"x: "<<cloud_in->points[i].x<<"\ty: "<<cloud_in->points[i].x<<"\tz: "<<cloud_in->points[i].z<<endl;

	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_rear.publish(cloud_to_pub);
}

void frontPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << front_tx, front_ty, front_tz;
//	transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
	transform.rotate(Eigen::AngleAxisf(front_pitch*M_PI/180, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_in, *cloud_in, transform);

	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-15,15);
	pass.filter(*cloud_in);
	
	cloud_in->header.frame_id = "base_link";

	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_in, cloud_to_pub);
    pub_front.publish(cloud_to_pub);
}

void configCallback(tools::lidar_calibrationConfig &config, uint32_t level)
{
	front_pitch = config.front_pitch;
	front_tx = config.front_tx;
	front_ty = config.front_ty;
	front_tz = config.front_tz;
	rear_roll = config.rear_roll;
	rear_pitch = config.rear_pitch;
	rear_yaw = config.rear_yaw;
	rear_tx = config.rear_tx;
	rear_ty = config.rear_ty;
	rear_tz = config.rear_tz;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lidar_calibration");
	ros::NodeHandle nh;
	ros::Subscriber sub_rear_cloud = nh.subscribe("/rear/velodyne_points",2,rearPointCloudCallback);
	ros::Subscriber sub_front_cloud = nh.subscribe("velodyne_points",2,frontPointCloudCallback);
	pub_rear = nh.advertise<sensor_msgs::PointCloud2>("rear_points_in_base_link",2);
	pub_front = nh.advertise<sensor_msgs::PointCloud2>("front_points_in_base_link",2);

	ros::NodeHandle pnh("~");
	pnh.param("front_pitch", front_pitch,0.0);
	pnh.param("front_tx", front_tx,0.0);
	pnh.param("front_ty", front_ty,0.0);
	pnh.param("front_tz", front_tz,0.0);
	pnh.param("rear_roll", rear_roll,0.0);
	pnh.param("rear_pitch", rear_pitch,0.0);
	pnh.param("rear_yaw", rear_yaw,0.0);
	pnh.param("rear_tx", rear_tx,0.0);
	pnh.param("rear_ty", rear_ty,0.0);
	pnh.param("rear_tz", rear_tz,0.0);


	//---------参数服务相关变量------------
	dynamic_reconfigure::Server<tools::lidar_calibrationConfig> dr_srv;
	dynamic_reconfigure::Server<tools::lidar_calibrationConfig>::CallbackType cb;
	//------配置动态更改参数服务-------
	cb = boost::bind(&configCallback, _1, _2);
	dr_srv.setCallback(cb);

	ros::spin();
	return 0;
}
