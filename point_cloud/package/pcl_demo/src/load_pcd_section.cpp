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

#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_demo/sectionConfig.h>
#include<deque>
using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);
int cnt = 2;
ros::Publisher pub_cloud;
ros::Publisher pub_map_marker;
int last_hi = 0;
int last_lo = 0;
bool first_time = true;
deque<int> size_array;
deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_deque;


void init_markers(visualization_msgs::Marker& marker)
{
	marker.header.frame_id = "map";
//	marker.header.stamp = ros::Time();
	marker.ns = "hehe";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
//	marker.lifetime = ros::Duration();
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
}

void add_marker_points(visualization_msgs::Marker& marker, double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  marker.points.push_back(p);
}

void add_marker_cloud(visualization_msgs::Marker& marker, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
	for(int i=0; i<cloud_in->size(); i++)
		add_marker_points(marker, cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
}

void add_marker_cloud(visualization_msgs::Marker& marker, deque<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_deque)
{
	for(int i=0; i<cloud_deque.size(); i++)
		for(int j=0; j<cloud_deque[i]->points.size(); j++)
			add_marker_points(marker, cloud_deque[i]->points[j].x, cloud_deque[i]->points[j].y, cloud_deque[i]->points[j].z);

}

void configCallback(pcl_demo::sectionConfig &config, uint32_t level)
{
	visualization_msgs::Marker marker;
	init_markers(marker);

	if((int)config.lo>(int)config.hi){
		config.lo = config.hi;
		cloud_deque.clear();
//		init_markers(marker);
		pub_map_marker.publish(marker);
		last_hi = (int)config.hi;
		last_lo = (int)config.lo;
		return;}


	if(first_time)
	{
		cloud_sum->clear();
		cnt = (int)config.lo;
		for(int i=(int)config.lo; i<(int)config.hi; i++)
		{
			std::stringstream ss;
			ss<<cnt++;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_in) == -1) //* load the file
			{
				std::cout<<"read error\n";
				break;
			}
			size_array.push_back(cloud_in->points.size());
			cloud_deque.push_back(cloud_in);
//			*cloud_sum += *cloud_in;
			ros::Duration(0.01).sleep();
		}

		first_time = false;
	}
	else
	{
		if( (int)config.hi > last_hi )
		{
			cnt = last_hi;
			for(int i=last_hi; i<(int)config.hi; i++)
			{
				std::stringstream ss;
				ss<<cnt++;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
				if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_in) == -1) //* load the file
				{
					std::cout<<"read error\n";break;
				}

				size_array.push_back(cloud_in->points.size());
				cloud_deque.push_back(cloud_in);
//				*cloud_sum += *cloud_in;
				ros::Duration(0.01).sleep();
			}
		}
		else if( (int)config.lo < last_lo )
		{
			cnt = last_lo;
			for(int i=last_lo; i>(int)config.lo; i--)
			{
				std::stringstream ss;
				ss<<cnt--;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
				if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_in) == -1) //* load the file
				{std::cout<<"read error\n";break;}

				size_array.push_front(cloud_in->points.size());
//				*cloud_sum += *cloud_in;
				cloud_deque.push_front(cloud_in);
				ros::Duration(0.01).sleep();
			}
		}
		else if( (int)config.hi < last_hi )
		{
			long int size_sum = 0;
			for(int i=(int)config.hi; i<last_hi; i++)
			{
    			size_sum += size_array.back();
    			size_array.pop_back();
    			cloud_deque.pop_back();
			}
//			cloud_sum->erase( cloud_sum->end()-size_sum, cloud_sum->end() );
		}
		else if( (int)config.lo > last_lo )
		{
			long int size_sum = 0;
			for(int i=last_lo; i<(int)config.lo; i++)
			{
    			size_sum += size_array.front();
    			size_array.pop_front();
    			cloud_deque.pop_front();
			}
//			cloud_sum->erase( cloud_sum->end()-size_sum, cloud_sum->end() );
		}
	}

	last_hi = (int)config.hi;
	last_lo = (int)config.lo;


	cloud_sum->header.frame_id = "map";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(*cloud_sum, cloud_to_pub);
	pub_cloud.publish(cloud_to_pub);

	add_marker_cloud(marker,cloud_deque);
	pub_map_marker.publish(marker);


}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "load_pcd");
	ros::NodeHandle nh;
	ros::Rate loop_rate(20);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("point_map", 10, true);
	pub_map_marker = nh.advertise<visualization_msgs::Marker>("map_marker", 10);

	pcl::PointXYZRGB pt;
	pt.x=0; pt.y=0;pt.z=0;pt.rgb=0;
	cloud_sum->push_back(pt);

	dynamic_reconfigure::Server<pcl_demo::sectionConfig> dr_srv;
	dynamic_reconfigure::Server<pcl_demo::sectionConfig>::CallbackType cb;

	cb = boost::bind(&configCallback, _1, _2);
	dr_srv.setCallback(cb);

	ros::spin();


	return 0;
}
