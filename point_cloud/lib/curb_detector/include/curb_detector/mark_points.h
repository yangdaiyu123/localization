#ifndef _MARK_POINTS_H_
#define	_MARK_POINTS_H_

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

const int	INVAILD_LABEL=	-100;

template <typename PointT>
void setPointsInvalid(pcl::PointCloud<PointT>& cloud)
{
	pcl::PointXYZI p;
	p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;
	cloud.clear();
	cloud.push_back(p);
}


template <typename PointT>
void markPoints(pcl::PointCloud<pcl::PointXYZRGB>& color_cloud, vector<pcl::PointCloud<PointT> > mark_points, int mark_color)
{
	pcl::PointXYZRGB p;
	for(int ring_id=0; ring_id<mark_points.size(); ring_id++)
	{
		for(int edge_id=0; edge_id<mark_points[ring_id].size(); edge_id++)
		{
			p.x = mark_points[ring_id].points[edge_id].x;
			p.y = mark_points[ring_id].points[edge_id].y;
			p.z = mark_points[ring_id].points[edge_id].z;
			p.rgb = *reinterpret_cast<float*>(&rainbow[mark_color]);
			color_cloud.push_back(p);
		}
	}
}


template <typename PointT>
void markPoints(pcl::PointCloud<pcl::PointXYZRGB>& color_cloud, pcl::PointCloud<PointT>  mark_points, int mark_color)
{
	if(mark_points.empty())	return;
	pcl::PointXYZRGB p;

	for(int i=0; i<mark_points.size(); i++)
	{
		p.x = mark_points.points[i].x;
		p.y = mark_points.points[i].y;
		p.z = mark_points.points[i].z;
		p.rgb = *reinterpret_cast<float*>(&rainbow[mark_color]);
		color_cloud.push_back(p);
	}
}

template <typename PointT>
void markOrientation(pcl::PointCloud<PointXYZO>& cloud, const pcl::PointCloud<PointT> inlier_points, const Eigen::VectorXf model)
{
	if(!inlier_points.empty())
	{
		PointXYZO po;
		double orientation = acos(model[3]);
		for(int i=0; i<inlier_points.points.size(); i++)
		{
			po.x = inlier_points.points[i].x;
			po.y = inlier_points.points[i].y;
			po.z = inlier_points.points[i].z;
			po.orientation = orientation;
			cloud.push_back(po);
		}
	}
}

Eigen::VectorXf modelConverter(const pcl::ModelCoefficients coefficients)
{
	Eigen::VectorXf model;
	for(int i=0; i<6; i++)
		model[i] = coefficients.values[i];
}

template <typename PointT>
bool is_point_available(PointT p)
{
	if(	int(p.z)==INVAILD_LABEL)
		return	false;
	else	return true;
}


}//namespace cloud_operation


#endif
