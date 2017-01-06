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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
int cnt = 0;
const double ODOMETRY_FACTOR = 0.0210386;

int pulse_sum = 0;
double odom_sum = 0.0;

tf::StampedTransform transform1;
tf::StampedTransform transform2;
tf::StampedTransform transform3;

geometry_msgs::PoseStamped pose;

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

            cloud_out->push_back(point_in_map);
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

            cloud_out->push_back(point_in_map);
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

            cloud_out->push_back(point_in_map);
        }

    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void pulseCallback(std_msgs::Int8MultiArray pulse_in)
{
	pulse_sum += ( pulse_in.data[0] + pulse_in.data[1] );
	odom_sum = pulse_sum * ODOMETRY_FACTOR;

//	cout<<"odom: "<<odom_sum<<endl;

	if(odom_sum > 50)
	{
		std::stringstream ss;
		ss<<cnt++;
		pcl::io::savePCDFileASCII ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_out);

		cloud_out->clear();
		pulse_sum = 0;
		odom_sum = 0;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "write_pcd");
	ros::NodeHandle nh;
	ros::Subscriber sub_front_curb = nh.subscribe("front_curb_raw",2,frontCurbCloudCallback);
	ros::Subscriber sub_rear_curb = nh.subscribe("rear_curb_raw",2,rearCurbCloudCallback);
	ros::Subscriber sub_sign = nh.subscribe("sign_points",2,signPointsCallback);
	ros::Subscriber sub_pulse = nh.subscribe("pulse",2,pulseCallback);

	ros::spin();
	return 0;
}
