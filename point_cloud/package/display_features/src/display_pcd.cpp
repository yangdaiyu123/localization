#include "ros/ros.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/segmentation/extract_clusters.h>

const int color_red =       0xff0000;
const int color_orange =    0xff8800;
const int color_yellow =    0xffff00;
const int color_green =     0x00ff00;
const int color_blue =      0x0000ff;
const int color_violet =    0xff00ff;
const int N_COLORS = 6;
int rainbow[N_COLORS] = {color_red, color_orange, color_yellow,
                       color_green, color_blue, color_violet};

int main (int argc, char** argv)
{
	ros::init(argc, argv, "display_pcd");
	ros::NodeHandle n;
	
	ros::Publisher pub_cloud = n.advertise<sensor_msgs::PointCloud2>("cloud_from_file",2);
	ros::Publisher pub_bin = n.advertise<sensor_msgs::PointCloud2>("cloud_binary",2);
	ros::Publisher pub_up = n.advertise<sensor_msgs::PointCloud2>("cloud_up_sample",2);
	ros::Publisher pub_cluster = n.advertise<sensor_msgs::PointCloud2>("cloud_clustered",2);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

	if (pcl::io::loadPCDFile<pcl::PointXYZI> ("arrow.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-3,7);
	pass.filter(*cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-4,4);
	pass.filter(*cloud);

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bin (new pcl::PointCloud<pcl::PointXYZI>);
	for(int i=0; i<cloud->points.size(); i++)
	{
		if(cloud->points[i].intensity>=10)
		{
			cloud->points[i].intensity = 200;
			cloud_bin->points.push_back(cloud->points[i]);
		}
		else
			cloud->points[i].intensity = 1;
	}
	
	pcl::RadiusOutlierRemoval<pcl::PointXYZI> filter;
	filter.setInputCloud(cloud_bin);
	filter.setRadiusSearch(0.5);
	filter.setMinNeighborsInRadius(5);
	filter.filter(*cloud_bin);
	
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI> filter_up;
	filter_up.setInputCloud(cloud_bin);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree;
	filter_up.setSearchMethod(kdtree);
	filter_up.setSearchRadius(0.2);
	filter_up.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointXYZI>::SAMPLE_LOCAL_PLANE);
	filter_up.setUpsamplingRadius(0.2);
	filter_up.setUpsamplingStepSize(0.1);
	filter_up.process(*cloud_filtered);
	
	
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud (cloud_filtered);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance (0.2); 
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (50000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
	  
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB p;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			p.x = cloud_filtered->points[*pit].x;
			p.y = cloud_filtered->points[*pit].y;
			p.z = cloud_filtered->points[*pit].z;
			p.rgb = *reinterpret_cast<float*>(&rainbow[j%6]);
			cloud_cluster->points.push_back (p);
		}
		j++;
	}

	
	cloud->header.frame_id = "base_link";
	cloud_bin->header.frame_id = "base_link";
	cloud_filtered->header.frame_id = "base_link";
	cloud_cluster->header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;

	ros::Rate r(10); // 10 hz
	while (ros::ok())
	{
		pcl::toROSMsg(*cloud, cloud_to_pub);
		pub_cloud.publish(cloud_to_pub);
		
		pcl::toROSMsg(*cloud_bin, cloud_to_pub);
		pub_bin.publish(cloud_to_pub);
		
		pcl::toROSMsg(*cloud_filtered, cloud_to_pub);
		pub_up.publish(cloud_to_pub);
		
		pcl::toROSMsg(*cloud_cluster, cloud_to_pub);
		pub_cluster.publish(cloud_to_pub);

		r.sleep();
	}



	return (0);
}
