#ifndef _FEATURE_CALC_H_
#define	_FEATURE_CALC_H_

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


template <typename PointT>
double getDevX(pcl::PointCloud<PointT> cloud, int start, int end)
{
	double sum_x=0.0;
	double mean_x;
	double dx;
	double sum_dx2=0.0;

	for(int i=start; i<end; i++)
		sum_x += cloud.points[i].x;

	mean_x = sum_x/(end-start);

	for(int i=start; i<end; i++)
	{
		dx = cloud.points[i].x - mean_x;
		sum_dx2 += dx*dx;
	}

//	cout<<"sum_x: "<<sum_x<<"\tmean_x: "<<mean_x<<"\tsum_dx2: "<<sum_dx2<<endl;

	return	sum_dx2;
}

template <typename PointT>
double getDevY(pcl::PointCloud<PointT> cloud, int start, int end)
{
	double sum_y=0.0;
	double mean_y;
	double dy;
	double sum_dy2=0.0;

	for(int i=start; i<end; i++)
		sum_y += cloud.points[i].y;

	mean_y = sum_y/(end-start);

	for(int i=start; i<end; i++)
	{
		dy = cloud.points[i].y - mean_y;
		sum_dy2 += dy*dy;
	}

//	cout<<"sum_y: "<<sum_y<<"\tmean_y: "<<mean_y<<"\tsum_dy2: "<<sum_dy2<<endl;

	return	sum_dy2;
}

template <typename PointT>
double getDevXYZ(pcl::PointCloud<PointT> cloud, int start, int end, double& dev_x, double& dev_y, double& dev_z)
{
	double sum_x=0.0,sum_y=0.0,sum_z=0.0;
	double mean_x=0.0,mean_y=0.0,mean_z=0.0;
	double dx,dy,dz;
	double sum_dx2=0.0,sum_dy2=0.0,sum_dz2=0.0;

	for(int i=start; i<end; i++)
	{
		sum_x += cloud.points[i].x;
		sum_y += cloud.points[i].y;
		sum_z += cloud.points[i].z;
	}

	mean_x = sum_x/(end-start);
	mean_y = sum_y/(end-start);
	mean_z = sum_z/(end-start);

	for(int i=start; i<end; i++)
	{
		dx = cloud.points[i].x - mean_x;
		sum_dx2 += dx*dx;
		dy = cloud.points[i].y - mean_y;
		sum_dy2 += dy*dy;
		dz = cloud.points[i].z - mean_z;
		sum_dz2 += dz*dz;
	}

	dev_x = sum_dx2;	dev_y = sum_dy2;	dev_z = sum_dz2;

	double ratio = dev_y / (dev_x * dev_z);
	return	sum_dy2;
}

template <typename PointT>
double getDevRatio(pcl::PointCloud<PointT> cloud, int start, int end)
{
	double dev_x = getDevX(cloud, start, end);
	double dev_y = getDevY(cloud, start, end);
	double ratio = dev_x/(dev_y+0.0001);
//	cout<<"x: "<<dev_x<<"\ty: "<<dev_y<<"\tratio: "<<ratio<<endl;
//	cout<<endl;
	return ratio;
}

template <typename PointT>
double getDevRatio(pcl::PointCloud<PointT> cloud, int start, int end, bool swap_x_y)
{
	double dev_x = getDevX(cloud, start, end);
	double dev_y = getDevY(cloud, start, end);
	double ratio = 0;
	if(swap_x_y)	ratio = dev_y/(dev_x+0.0001);
	else	ratio = dev_x/(dev_y+0.0001);
	return ratio;
}


}//namespace cloud_operation


#endif
