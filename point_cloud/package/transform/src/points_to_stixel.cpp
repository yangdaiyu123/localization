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

const int GRID_HEIGHT = 500;
const int GRID_WIDTH = 500;


pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher	pub_cloud;

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

double get_min_height( pcl::PointCloud<pcl::PointXYZI> cloud )
{
	if( cloud.empty() ) return 0;
	
	double min_h = 10.0;
	for( int i=0; i<cloud.points.size(); i++ )
		if( min_h > cloud.points[i].z )
			min_h = cloud.points[i].z;
	
	return min_h; 
}

double min_max_limit( double val, double min, double max )
{
	val = val < min ? min : val;
	val = val > max ? max : val;
	return val;
}

void cut_x_y( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double x_min, double x_max, double y_min, double y_max )
{
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(x_min,x_max);
	pass.filter(*cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y_min,y_max);
	pass.filter(*cloud);
}

void cut_z( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double z_min, double z_max )
{
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(z_min,z_max);
	pass.filter(*cloud);
}


void lidarCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::fromROSMsg(*input, *lidar_points);
	
	cut_x_y( lidar_points, -50, 50, -50, 50 );
	
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

	cloud_filtered->clear();
	for(int i=0; i<GRID_HEIGHT; i++)
		for(int j=0; j<GRID_WIDTH; j++)
			{
				if( cloud_grid_array[i][j].points.size() > 20 )
				{
					double max_h = get_max_height( cloud_grid_array[i][j] );
					double min_h = get_min_height( cloud_grid_array[i][j] );
					if( max_h > 2.5 && min_h < 1.5 )
						cloud_grid_array[i][j].header.seq = 1;
				}
			}

	for(int i=1; i<GRID_HEIGHT-1; i++)
		for(int j=1; j<GRID_WIDTH-1; j++)
			{
				if( cloud_grid_array[i][j].header.seq == 1 )
				{
					pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZI>);
					cloud_blob->clear();
					for( int ii=-1; ii<=1; ii++ )
						for( int jj=-1; jj<=1; jj++ )
							*cloud_blob += cloud_grid_array[i+ii][j+jj];
					
					double max_h = get_max_height( cloud_grid_array[i][j] );
					double min_h = get_min_height( cloud_grid_array[i][j] );
					double diff_h = max_h - min_h;
					max_h -= diff_h/4;
					min_h += diff_h/4;
					cut_z( cloud_blob, min_h, max_h );
					double occ_ratio = cloud_grid_array[i][j].points.size()*1.0/ cloud_blob->points.size();
					if( occ_ratio < 0.8 ) 
						cloud_grid_array[i][j].header.seq = 0;
						
					double dist = sqrtf( pow(cloud_grid_array[i][j].points[0].x, 2) + pow(cloud_grid_array[i][j].points[0].y, 2) );
					if( abs(cloud_grid_array[i][j].points[0].y) > 15 )
						cloud_grid_array[i][j].header.seq = 0;
						
					vector<float> vech;
					for(int idx=0; idx<cloud_grid_array[i][j].points.size(); idx++)
						vech.push_back( cloud_grid_array[i][j].points[idx].z );
					sort( vech.begin(), vech.end() );
					for(int it=1; it<vech.size(); it++)
						if( (vech[it] - vech[it-1]) > 0.6 && vech[it-1] < 1.5 )
						{
							cloud_grid_array[i][j].header.seq = 0;
							break;
						}	
				}
			}

	for(int i=1; i<GRID_HEIGHT-1; i++)
		for(int j=1; j<GRID_WIDTH-1; j++)
				if( cloud_grid_array[i][j].header.seq == 1 )
					*cloud_filtered += cloud_grid_array[i][j];

	cloud_filtered->header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_filtered, cloud_to_pub);
    pub_cloud.publish(cloud_to_pub);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_to_stixel");
	ros::NodeHandle nh;

	ros::Subscriber sub_lidar =  nh.subscribe("points_in_base_link", 2, &lidarCloudCallback);
	pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("stixel_cloud",2);
    
	ros::spin();
	return 0;
}
