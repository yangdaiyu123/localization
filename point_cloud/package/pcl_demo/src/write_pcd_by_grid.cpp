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

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <sstream>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace message_filters;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_curb(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sign(new pcl::PointCloud<pcl::PointXYZRGB>);

int cnt = 0;

tf::StampedTransform transform1;
tf::StampedTransform transform2;
tf::StampedTransform transform3;

geometry_msgs::PoseStamped pose;
vector<tf::Vector3> grid_array;
tf::Vector3 grid(0,0,0);

const int color_red =       0xff0000;
const int color_orange =    0xff8800;
const int color_yellow =    0xffff00;
const int color_green =     0x00ff00;
const int color_blue =      0x0000ff;
const int color_violet =    0xff00ff;


void frontCurbCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	try
	{
		static tf::TransformListener trf_listener1;
        trf_listener1.lookupTransform("map", "base_link", ros::Time(0), transform1);

        for (int i=0; i<cloud_in->size(); i++)
        {
            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            tf::Vector3 converted = transform1 * pt;

            pcl::PointXYZRGB point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.rgb = color_green;

            cloud_curb->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void rearCurbCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);

	try
	{
		static tf::TransformListener trf_listener2;
        trf_listener2.lookupTransform("map", "base_link", ros::Time(0), transform2);

        for (int i=0; i<cloud_in->size(); i++)
        {
            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            tf::Vector3 converted = transform2 * pt;

            pcl::PointXYZRGB point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.rgb = color_green;

            cloud_curb->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void signPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::fromROSMsg(*msg, *cloud_in);
	try
	{
		static tf::TransformListener trf_listener3;
        trf_listener3.lookupTransform("map", "base_link", ros::Time(0), transform3);

        for (int i=0; i<cloud_in->size(); i++)
        {
            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            tf::Vector3 converted = transform3 * pt;

            pcl::PointXYZRGB point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.rgb = color_yellow;

            cloud_sign->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void gpsPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	pose = pose_in;

	*cloud_out = *cloud_curb + *cloud_sign;

	cout<<"x: "<<int(pose_in.pose.position.x/50)<<"\ty: "<<int(pose_in.pose.position.y/50)<<endl;


	int grid_x = int(pose_in.pose.position.x/50);
	int grid_y = int(pose_in.pose.position.y/50);



	if(grid.z() == 0)
	{
		grid.setX(grid_x);
		grid.setY(grid_y);
		grid.setZ(1);
	}
	else
	{
		if(grid_x != grid.x() || grid_y != grid.y() )
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_old(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::stringstream ss;
			ss<<grid.x()<<"_"<<grid.y()<<".pcd"<<endl;
			cout<<ss.str();

			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_old) == -1)
				if(!cloud_out->empty())
					pcl::io::savePCDFileASCII ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_out);
			else
			{
				*cloud_out += *cloud_old;
				if(!cloud_out->empty())
					pcl::io::savePCDFileASCII ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_out);
			}

			cloud_out->clear();
			grid.setX(grid_x);
			grid.setY(grid_y);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "write_pcd");
	ros::NodeHandle nh;
	ros::Subscriber sub_front_curb = nh.subscribe("front_curb_raw",2,frontCurbCloudCallback);
	ros::Subscriber sub_rear_curb = nh.subscribe("rear_curb_raw",2,rearCurbCloudCallback);
	ros::Subscriber sub_sign = nh.subscribe("sign_points",2,signPointsCallback);
	ros::Subscriber sub_gps_pose = nh.subscribe("gps_pose",2,gpsPoseCallback);

	ros::spin();
	return 0;
}
