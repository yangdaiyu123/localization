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
int grid_scale=50;

double travel_distance = 0;
const double ODOMETRY_FACTOR = 0.0210386;
int odom_section =0;
int last_odom_section = -1;
int odom_scale=50;

template <typename PointT>
void publish_map(ros::Publisher pub, pcl::PointCloud<PointT> cloud, const string frame_id)
{
	cloud.header.frame_id = frame_id;
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(cloud, cloud_to_pub);
	pub.publish(cloud_to_pub);
}



void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	travel_distance += odom_inc;
	
	odom_section = travel_distance / odom_scale;
	
	if( odom_section ==  last_odom_section )
		return;
	
	cloud_sum->clear();
	
	pcl::PointCloud<PointXYZO>::Ptr cloud_in(new pcl::PointCloud<PointXYZO>);

	for(int i=-4; i<=1; i++)
	{
		std::stringstream ss;
		ss<<odom_section+i<<".pcd";
		if (pcl::io::loadPCDFile<PointXYZO> (map_path+ss.str(), *cloud_in) == -1)
			std::cout<<"read error\n";
		else *cloud_sum += *cloud_in;
	}
	
	last_odom_section = odom_section;

}

void timerCallback(const ros::TimerEvent&)
{
	publish_map(pub_cloud, *cloud_sum, "map");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "load_pcd_odom");

	ros::NodeHandle nh;
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("point_map", 10, true);

	ros::NodeHandle pnh("~");
	pnh.param<std::string>("map_path", map_path, "/home/wlh/map/oriented_point_map/");
	pnh.param<std::string>("pose_type", pose_type, "/truth_pose");
	pnh.param("odom_scale", odom_scale, 50);
	ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, pulseCallback);
	ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
	ros::spin();


	return 0;
}
