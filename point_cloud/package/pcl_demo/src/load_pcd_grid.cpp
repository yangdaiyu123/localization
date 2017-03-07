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

#include <pcl_ros/point_cloud.h>
#include <point_xyzo.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <sstream>
#include <string>

#include<deque>
using namespace std;

pcl::PointCloud<PointXYZO>::Ptr cloud_sum(new pcl::PointCloud<PointXYZO>);
ros::Publisher pub_cloud;

int last_grid_x = 0;
int last_grid_y = 0;

string map_path;
string pose_type;
int grid_scale;

template <typename PointT>
void publish_map(ros::Publisher pub, pcl::PointCloud<PointT> cloud, const string frame_id)
{
	cloud.header.frame_id = frame_id;
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(cloud, cloud_to_pub);
	pub.publish(cloud_to_pub);
}

void gpsPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	int grid_x = int(pose_in.pose.position.x/grid_scale);
	int grid_y = int(pose_in.pose.position.y/grid_scale);

	cout<<"grid_x: "<<grid_x<<"\t"<<"grid_y: "<<grid_y<<endl;

	if(grid_x ==  last_grid_x && grid_y == last_grid_y)
	{
		publish_map(pub_cloud, *cloud_sum, "map");
		return;
	}

	cloud_sum->clear();

//	load_nearby_map(cloud_sum, grid_x, grid_y);
	pcl::PointCloud<PointXYZO>::Ptr cloud_in(new pcl::PointCloud<PointXYZO>);

	for(int i=-1; i<=1; i++)
		for(int j=-1; j<=1; j++)
		{
			std::stringstream ss;
			ss<<grid_x+i<<"_"<<grid_y+j<<".pcd"<<endl;
			if (pcl::io::loadPCDFile<PointXYZO> (map_path+ss.str()+".pcd", *cloud_in) == -1)
				std::cout<<"read error\n";
			else *cloud_sum += *cloud_in;
		}

	publish_map(pub_cloud, *cloud_sum, "map");

	last_grid_x = grid_x;
	last_grid_y = grid_y;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "load_pcd");

	ros::NodeHandle nh;
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("point_map", 10, true);

	ros::NodeHandle pnh("~");
	pnh.param<std::string>("map_path", map_path, "/home/wlh/map/oriented_point_map/");
	pnh.param<std::string>("pose_type", pose_type, "/truth_pose");
	pnh.param("grid_scale", grid_scale, 50);
	ros::Subscriber sub_gps_pose = nh.subscribe(pose_type,2,gpsPoseCallback);

	ros::spin();


	return 0;
}
