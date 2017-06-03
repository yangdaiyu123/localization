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
#include <geometry_msgs/PoseStamped.h>

#include <pcl_ros/point_cloud.h>
#include <point_xyzo.h>

#include <sstream>
#include <string>

using namespace std;

pcl::PointCloud<PointXYZO>::Ptr cloud_in(new pcl::PointCloud<PointXYZO>);
pcl::PointCloud<PointXYZO>::Ptr cloud_out(new pcl::PointCloud<PointXYZO>);
pcl::PointCloud<PointXYZO>::Ptr cloud_curb(new pcl::PointCloud<PointXYZO>);
pcl::PointCloud<PointXYZO>::Ptr cloud_sign(new pcl::PointCloud<PointXYZO>);
pcl::PointCloud<PointXYZO>::Ptr cloud_marker(new pcl::PointCloud<PointXYZO>);
pcl::PointCloud<PointXYZO>::Ptr cloud_pole(new pcl::PointCloud<PointXYZO>);

tf::StampedTransform transform1;
tf::StampedTransform transform2;
tf::StampedTransform transform3;
tf::StampedTransform transform4;
tf::StampedTransform transform5;

geometry_msgs::PoseStamped pose;
vector<tf::Vector3> grid_array;
tf::Vector3 grid(0,0,0);

string map_path;
int grid_scale = 50;

double travel_distance = 0;
const double ODOMETRY_FACTOR = 0.0210386;
int odom_section, last_odom_section;
int odom_scale=50;
bool first_time = true;

void frontCurbCloudCallback(const OPointCloud::ConstPtr &cloud_in)
{
	try
	{
		static tf::TransformListener trf_listener1;
        trf_listener1.lookupTransform("map", "base_link", ros::Time(0), transform1);

        for (int i=0; i<cloud_in->size(); i++)
        {
            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            tf::Vector3 converted = transform1 * pt;

            PointXYZO point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.orientation = cloud_in->points[i].orientation;

            cloud_curb->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void rearCurbCloudCallback(const OPointCloud::ConstPtr &cloud_in)
{
	try
	{
		static tf::TransformListener trf_listener2;
        trf_listener2.lookupTransform("map", "base_link", ros::Time(0), transform2);

        for (int i=0; i<cloud_in->size(); i++)
        {
            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            tf::Vector3 converted = transform2 * pt;

            PointXYZO point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.orientation = cloud_in->points[i].orientation;

            cloud_curb->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void signPointsCallback(const OPointCloud::ConstPtr &cloud_in)
{
	try
	{
		static tf::TransformListener trf_listener3;
        trf_listener3.lookupTransform("map", "base_link", ros::Time(0), transform3);

        for (int i=0; i<cloud_in->size(); i++)
        {
            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
            tf::Vector3 converted = transform3 * pt;

            PointXYZO point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.orientation = cloud_in->points[i].orientation;

            cloud_sign->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}

void markerPointsCallback(const OPointCloud::ConstPtr &cloud_in)
{	
//	cout<<"receive marker"<<endl;
//	try
//	{
//		static tf::TransformListener trf_listener4;
//        trf_listener4.lookupTransform("map", "base_link", ros::Time(0), transform4);

//        for (int i=0; i<cloud_in->size(); i++)
//        {
//            tf::Vector3 pt(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
//            tf::Vector3 converted = transform4 * pt;

//            PointXYZO point_in_map;
//            point_in_map.x = converted.x();
//            point_in_map.y = converted.y();
//            point_in_map.z = converted.z();
//            point_in_map.orientation = 10;

//            cloud_marker->push_back(point_in_map);
//        }
//    }
//    catch (tf::TransformException ex) {
//        ROS_INFO("%s", ex.what());
//        ros::Duration(0.01).sleep();
//    }
}

void poleCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pole_in(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*cloud_in, *cloud_pole_in);
	std::cout<<"receive pole"<<std::endl;
	try
	{
		static tf::TransformListener trf_listener5;
        trf_listener5.lookupTransform("map", "base_link", ros::Time(0), transform5);

        for (int i=0; i<cloud_pole_in->size(); i++)
        {
            tf::Vector3 pt(cloud_pole_in->points[i].x, cloud_pole_in->points[i].y, cloud_pole_in->points[i].z);
            tf::Vector3 converted = transform5 * pt;

            PointXYZO point_in_map;
            point_in_map.x = converted.x();
            point_in_map.y = converted.y();
            point_in_map.z = converted.z();
            point_in_map.orientation = 0;

            cloud_pole->push_back(point_in_map);
        }
    }
    catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}


void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	travel_distance += odom_inc;
	
	odom_section = travel_distance / odom_scale;

	*cloud_out = *cloud_curb + *cloud_sign;
	
	*cloud_out += *cloud_pole;
	
//	cout<<"odom: "<<travel_distance<<" "<<"odom_section: "<<odom_section<<endl;


	if(first_time)
	{
		last_odom_section = odom_section;
		first_time = false;
		return;
	}
	else
	{
		if( odom_section != last_odom_section )
		{
			pcl::PointCloud<PointXYZO>::Ptr cloud_old(new pcl::PointCloud<PointXYZO>);
			std::stringstream ss;
			ss<<last_odom_section<<".pcd";
			cout<<ss.str();

			pcl::io::savePCDFileASCII (map_path + ss.str(), *cloud_out);
			

			cloud_out->clear();
			cloud_curb->clear();
			cloud_sign->clear();
			cloud_pole->clear();
		}
	}
	
	last_odom_section = odom_section;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "write_pcd_odom");
	ros::NodeHandle nh;
	ros::Subscriber sub_front_curb = nh.subscribe("front_curb_raw",2,frontCurbCloudCallback);
	ros::Subscriber sub_rear_curb = nh.subscribe("rear_curb_raw",2,rearCurbCloudCallback);
	ros::Subscriber sub_sign = nh.subscribe("sign_points",2,signPointsCallback);
	ros::Subscriber sub_marker = nh.subscribe("marker_points",2,markerPointsCallback);
	ros::Subscriber sub_pole = nh.subscribe("stixel_cloud",2,poleCallback);

	ros::NodeHandle pnh("~");
	pnh.param<std::string>("map_path", map_path, "/home/wlh/map/oriented_point_map/");
	pnh.param("odom_scale", odom_scale, 50);
	
	ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, pulseCallback);
	

	ros::spin();
	return 0;
}
