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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus//model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <point_xyzo.h>

#include <sstream>
#include <string>

using namespace std;

// RGB color values
const int color_red =       0xff0000;
const int color_orange =    0xff8800;
const int color_yellow =    0xffff00;
const int color_green =     0x00ff00;
const int color_blue =      0x0000ff;
const int color_violet =    0xff00ff;

const int N_COLORS = 6;
int rainbow[N_COLORS] = {color_red, color_orange, color_yellow,
                       color_green, color_blue, color_violet};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr curb_front(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr curb_rear(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_curb(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Publisher pub_left_line;
ros::Publisher pub_right_line;

void frontCurbCloudCallback(const OPointCloud::ConstPtr &cloud_in)
{
	pcl::PointXYZRGB pt;
	for( int i=0; i<cloud_in->points.size(); i++ )
	{
		pt.x = cloud_in->points[i].x;
		pt.y = cloud_in->points[i].y;
		pt.z = cloud_in->points[i].z;
		pt.rgb = *reinterpret_cast<float*>(&rainbow[2]);
		
		curb_front->points.push_back(pt);
	}
	
	*cloud_curb = *curb_front + *curb_rear;
	
	pcl::PointCloud<pcl::PointXYZRGB> left_curb, right_curb;
	pcl::PointCloud<pcl::PointXYZRGB> left_line, right_line;
	
	for(int i=0; i< cloud_curb->points.size(); i++ )
	{
		if( cloud_curb->points[i].y > 0 )
			left_curb.points.push_back(cloud_curb->points[i]);
		else
			right_curb.points.push_back(cloud_curb->points[i]);
	}
		
	if( left_curb.points.size() > 10 )
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//创建一个模型参数对象，用于记录结果
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inliers表示误差能容忍的点 记录的是点云的序号
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;// 创建一个分割器
		seg.setOptimizeCoefficients (true);// Optional
		seg.setModelType (pcl::SACMODEL_LINE);// Mandatory-设置目标几何形状
		seg.setMethodType (pcl::SAC_RANSAC);//分割方法：随机采样法
		seg.setDistanceThreshold (0.2);//设置误差容忍范围
		seg.setMaxIterations(80);
	
		seg.setInputCloud (left_curb.makeShared());

		seg.segment (*inliers, *coefficients);
	
		for(int i=0; i<6; i++)
			cout<<coefficients->values[i]<<" ";
		
		cout<<endl;
		
		pcl::PointXYZRGB p;
		for( int i=0; i<500; i++ )
		{
			p.x = coefficients->values[0] - i*coefficients->values[3] * 0.1;
			p.y = coefficients->values[1] - i*coefficients->values[4] * 0.1;
			p.z = coefficients->values[2] - i*coefficients->values[5] * 0.1;
			p.rgb = *reinterpret_cast<float*>(&rainbow[3]);
			left_curb.points.push_back(p);
		}
		for( int i=0; i<500; i++ )
		{
			p.x = coefficients->values[0] + i*coefficients->values[3] * 0.1;
			p.y = coefficients->values[1] + i*coefficients->values[4] * 0.1;
			p.z = coefficients->values[2] + i*coefficients->values[5] * 0.1;
			p.rgb = *reinterpret_cast<float*>(&rainbow[3]);
			left_curb.points.push_back(p);
		}
	}
	
	if( right_curb.points.size() > 10 )
	{
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//创建一个模型参数对象，用于记录结果
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inliers表示误差能容忍的点 记录的是点云的序号
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;// 创建一个分割器
		seg.setOptimizeCoefficients (true);// Optional
		seg.setModelType (pcl::SACMODEL_LINE);// Mandatory-设置目标几何形状
		seg.setMethodType (pcl::SAC_RANSAC);//分割方法：随机采样法
		seg.setDistanceThreshold (0.2);//设置误差容忍范围
		seg.setMaxIterations(80);
	
		seg.setInputCloud (right_curb.makeShared());

		seg.segment (*inliers, *coefficients);
	
		for(int i=0; i<6; i++)
			cout<<coefficients->values[i]<<" ";
		
		cout<<endl;
		
		pcl::PointXYZRGB p;
		for( int i=0; i<500; i++ )
		{
			p.x = coefficients->values[0] - i*coefficients->values[3] * 0.1;
			p.y = coefficients->values[1] - i*coefficients->values[4] * 0.1;
			p.z = coefficients->values[2] - i*coefficients->values[5] * 0.1;
			p.rgb = *reinterpret_cast<float*>(&rainbow[3]);
			right_curb.points.push_back(p);
		}
		for( int i=0; i<500; i++ )
		{
			p.x = coefficients->values[0] + i*coefficients->values[3] * 0.1;
			p.y = coefficients->values[1] + i*coefficients->values[4] * 0.1;
			p.z = coefficients->values[2] + i*coefficients->values[5] * 0.1;
			p.rgb = *reinterpret_cast<float*>(&rainbow[3]);
			right_curb.points.push_back(p);
		}
		
	}
	
	
	left_curb.header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(left_curb, cloud_to_pub);
	pub_left_line.publish(cloud_to_pub);
	
	right_curb.header.frame_id = "base_link";
	pcl::toROSMsg(right_curb, cloud_to_pub);
	pub_right_line.publish(cloud_to_pub);
	
	curb_front->clear();
	curb_rear->clear();
	cloud_curb->clear();
}

void rearCurbCloudCallback(const OPointCloud::ConstPtr &cloud_in)
{
	pcl::PointXYZRGB pt;
	for( int i=0; i<cloud_in->points.size(); i++ )
	{
		pt.x = cloud_in->points[i].x;
		pt.y = cloud_in->points[i].y;
		pt.z = cloud_in->points[i].z;
		pt.rgb = *reinterpret_cast<float*>(&rainbow[2]);
		
		curb_rear->points.push_back(pt);
	}
}





int main(int argc, char **argv)
{
	ros::init(argc, argv, "display_fitting_curb");
	ros::NodeHandle nh;
	ros::Subscriber sub_front_curb = nh.subscribe("front_curb_raw",2,frontCurbCloudCallback);
	ros::Subscriber sub_rear_curb = nh.subscribe("rear_curb_raw",2,rearCurbCloudCallback);
	
	pub_left_line = nh.advertise<sensor_msgs::PointCloud2>("left_line",2);
	pub_right_line = nh.advertise<sensor_msgs::PointCloud2>("right_line",2);

	ros::spin();
	return 0;
}
