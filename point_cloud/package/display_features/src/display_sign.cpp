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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_sign(new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Publisher pub_color_sign;

void signPointsCloudCallback(const OPointCloud::ConstPtr &cloud_in)
{
	pcl::PointXYZRGB pt;
	for( int i=0; i<cloud_in->points.size(); i++ )
	{
		pt.x = cloud_in->points[i].x;
		pt.y = cloud_in->points[i].y;
		pt.z = cloud_in->points[i].z;
		pt.rgb = *reinterpret_cast<float*>(&rainbow[2]);
		
		if( pt.z > 4 )
			color_sign->points.push_back(pt);
	}
	
	color_sign->header.frame_id = "map";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(*color_sign, cloud_to_pub);
	pub_color_sign.publish(cloud_to_pub);

	color_sign->clear();
}





int main(int argc, char **argv)
{
	ros::init(argc, argv, "display_sign");
	ros::NodeHandle nh;
	ros::Subscriber sub_sign = nh.subscribe("point_map",2,signPointsCloudCallback);
	
	pub_color_sign = nh.advertise<sensor_msgs::PointCloud2>("color_sign",2);

	ros::spin();
	return 0;
}
