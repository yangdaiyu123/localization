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

tf::StampedTransform transform1;
tf::StampedTransform transform2;

geometry_msgs::PoseStamped pose;
vector<tf::Vector3> grid_array;
tf::Vector3 grid(0,0,0);

string map_path;
int grid_scale;

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


void gpsPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	pose = pose_in;

	*cloud_out = *cloud_curb;

	cout<<"x: "<<int(pose_in.pose.position.x/grid_scale)<<"\ty: "<<int(pose_in.pose.position.y/50)<<endl;

	int grid_x = int(pose_in.pose.position.x/grid_scale);
	int grid_y = int(pose_in.pose.position.y/grid_scale);

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
			pcl::PointCloud<PointXYZO>::Ptr cloud_old(new pcl::PointCloud<PointXYZO>);
			std::stringstream ss;
			ss<<grid.x()<<"_"<<grid.y()<<".pcd";
			cout<<ss.str();

			if (pcl::io::loadPCDFile<PointXYZO> (map_path + ss.str(), *cloud_old) == -1)
				if(!cloud_out->empty())
					pcl::io::savePCDFileASCII (map_path + ss.str(), *cloud_out);
			else
			{
				*cloud_out += *cloud_old;
				if(!cloud_out->empty())
					pcl::io::savePCDFileASCII (map_path + ss.str(), *cloud_out);
			}

			cloud_out->clear();
			cloud_curb->clear();
			grid.setX(grid_x);
			grid.setY(grid_y);
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "write_pcd_oriented_grid");
	ros::NodeHandle nh;
	ros::Subscriber sub_front_curb = nh.subscribe("front_curb_raw",2,frontCurbCloudCallback);
	ros::Subscriber sub_rear_curb = nh.subscribe("rear_curb_raw",2,rearCurbCloudCallback);
	ros::Subscriber sub_gps_pose = nh.subscribe("truth_pose",2,gpsPoseCallback);

	ros::NodeHandle pnh("~");
	pnh.param<std::string>("map_path", map_path, "/home/wlh/map/curb_point_map/");
	pnh.param("grid_scale", grid_scale, 50);

	ros::spin();
	return 0;
}
