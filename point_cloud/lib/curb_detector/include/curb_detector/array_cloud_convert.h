#ifndef _ARRAY_CLOUD_CONVErT_H_
#define	_ARRAY_CLOUD_CONVErT_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>


#include <point_types.h>
#include <point_xyzo.h>
#include <pcl_ros/point_cloud.h>
#include <ransac_line/ransac_line.h>

#include <string>
#include <iostream>
#include <fstream>

namespace cloud_operation
{

using namespace std;
using namespace pcl;

vector<pcl::PointCloud<pcl::PointXYZI> > cloudToArray(const VPointCloud::ConstPtr &inMsg,const int ARRAY_SIZE)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array;
	cloud_array.resize(ARRAY_SIZE);
	for (int i = 1; i < inMsg->points.size(); i++)
	{
		if(inMsg->points[i].ring < ARRAY_SIZE)
		{
			int ring_idx = inMsg->points[i].ring;
			pcl::PointXYZI	pt;
			pt.x = inMsg->points[i].x;
			pt.y = inMsg->points[i].y;
			pt.z = inMsg->points[i].z;
			pt.intensity = inMsg->points[i].intensity;

			cloud_array[ring_idx].push_back(pt);
		}
	}
	return cloud_array;
}


pcl::PointCloud<pcl::PointXYZI> arrayToCloudXYZI(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI p;
	for(int ring_id=0; ring_id<cloud_array.size(); ring_id++)
	{
		for(int point_id=0; point_id < cloud_array[ring_id].points.size(); point_id++)
		{
			p.x = cloud_array[ring_id].points[point_id].x;
			p.y = cloud_array[ring_id].points[point_id].y;
			p.z = cloud_array[ring_id].points[point_id].z;
			p.intensity = cloud_array[ring_id].points[point_id].z;
			cloud.push_back(p);
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ> arrayToCloudXYZ(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{
	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::PointXYZ p;
	for(int ring_id=0; ring_id<cloud_array.size(); ring_id++)
	{
		for(int point_id=0; point_id < cloud_array[ring_id].points.size(); point_id++)
		{
			p.x = cloud_array[ring_id].points[point_id].x;
			p.y = cloud_array[ring_id].points[point_id].y;
			p.z = cloud_array[ring_id].points[point_id].z;
			cloud.push_back(p);
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB> arrayToColorCloud(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array, int color)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::PointXYZRGB p;
	for(int ring_id=0; ring_id<cloud_array.size(); ring_id++)
	{
		for(int point_id=0; point_id < cloud_array[ring_id].points.size(); point_id++)
		{
			p.x = cloud_array[ring_id].points[point_id].x;
			p.y = cloud_array[ring_id].points[point_id].y;
			p.z = cloud_array[ring_id].points[point_id].z;
			p.rgb = *reinterpret_cast<float*>(&rainbow[color]);
			cloud.push_back(p);
		}
	}
	return cloud;
}

std::vector<pcl::PointXYZ> cloudToArray(const pcl::PointCloud<pcl::PointXYZ> cloud)
{
	std::vector<pcl::PointXYZ> points_set;
	for(int i=0; i<cloud.size(); i++)
		points_set.push_back(cloud.points[i]);
	return points_set;
}

std::vector<pcl::PointCloud<pcl::PointXYZI> > cloudToCircularBlocks(const pcl::PointCloud<pcl::PointXYZI> input)
{
	std::vector<pcl::PointCloud<pcl::PointXYZI> > circular_blocks;


	return circular_blocks;
}


}//namespace cloud_operation


#endif
