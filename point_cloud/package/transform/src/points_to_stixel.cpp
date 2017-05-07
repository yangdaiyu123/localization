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
#include <pcl/filters/random_sample.h>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>


#include <vector>


using namespace std;
using namespace cv;

const int GRID_HEIGHT = 500;
const int GRID_WIDTH = 500;


pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZI>);


double get_ave_intensity(pcl::PointCloud<pcl::PointXYZI> cloud)
{
	if( cloud.empty() ) return 0;

	double intensity_sum = 0.0, intensity_ave = 0.0;
	for(int i=0; i<cloud.points.size(); i++)
		intensity_sum += cloud.points[i].intensity;
	intensity_ave = intensity_sum / cloud.points.size();
	
	return intensity_ave;
}

double get_max_height( pcl::PointCloud<pcl::PointXYZI> cloud )
{
	if( cloud.empty() ) return 0;
	
	double max_h = 0.0;
	for( int i=0; i<cloud.points.size(); i++ )
		if( max_h < cloud.points[i].z )
			max_h = cloud.points[i].z;
	
	return max_h; 
}

double min_max_limit( double val, double min, double max )
{
	val = val < min ? min : val;
	val = val > max ? max : val;
	return val;
}


void lidarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::fromROSMsg(*input, *lidar_points);
	
	vector<vector<pcl::PointCloud<pcl::PointXYZI> > > cloud_grid_array;
	cloud_grid_array.resize(500);
	for (int i = 0; i < 500; ++i)
    	cloud_grid_array[i].resize(500);

	for (int i = 0; i < lidar_points->size(); i++)
	{
		int colum = 0;
		int row = 0;

		colum = int( ( 50 - lidar_points->points[i].y ) / 0.2);
		row = int( ( 50 - lidar_points->points[i].x ) / 0.2);

		if (row >= GRID_HEIGHT) row = GRID_HEIGHT-1;
		if (colum >= GRID_WIDTH) colum = GRID_WIDTH-1;

		cloud_grid_array[row][colum].points.push_back(lidar_points->points[i]);

	}


	for(int i=0; i<GRID_HEIGHT; i++)
		for(int j=0; j<GRID_WIDTH; j++)
			{}

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_to_stixel");
	ros::NodeHandle nh;

	ros::Subscriber sub_lidar =  nh.subscribe("points_in_base_link", 2, &lidarCloudCallback);

    
	ros::spin();
	return 0;
}
