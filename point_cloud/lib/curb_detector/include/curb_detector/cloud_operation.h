#ifndef _CLOUD_OPERATION_H
#define	_CLOUD_OPERATION_H

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

#include <mark_points.h>
#include <array_cloud_convert.h>
#include <cloud_sort.h>
#include <feature_calc.h>
#include <cloud_search.h>


namespace cloud_operation
{

using namespace std;
using namespace pcl;



namespace filter
{

template <typename PointT>
void remove_invalid_points(pcl::PointCloud<PointT>&  cloud)
{
	for (typename pcl::PointCloud<PointT>::iterator it = cloud.begin(); it != cloud.end(); /**/)
	    if ( !is_point_available(*it) ) it = cloud.erase(it);
	    else ++it;
}


template <typename PointT>
bool is_position_reasonable(PointT p)
{
	if(p.z>0.5)	return false;
	else if(abs(p.y)>8) return false;
	else return true;
}

template <typename PointT>
void dist_filter(pcl::PointCloud<PointT>&  cloud)
{
	for (typename pcl::PointCloud<PointT>::iterator it = cloud.begin(); it != cloud.end(); /**/)
		if ( !is_position_reasonable(*it) ) it = cloud.erase(it);
	    else ++it;
}

pcl::PointCloud<pcl::PointXYZ> ransac_fit_line(pcl::PointCloud<pcl::PointXYZ> edges)
{
	std::vector<pcl::PointXYZ> points_set = cloudToArray(edges);
	pcl::PointCloud<pcl::PointXYZ> inlier_points;

	if(points_set.size()<40){inlier_points.clear();return inlier_points;}

	std::vector<int> inliers_idx;
	Ransac_Line Line;
	Line.setInputCloud(points_set);
	Line.setDistanceThreshold(0.3);
	Line.computeModel();
	Line.getInliers(inliers_idx);

	for(int i=0; i<inliers_idx.size(); i++)
		inlier_points.push_back(points_set[inliers_idx[i]]);

	return	inlier_points;
}

pcl::PointCloud<pcl::PointXYZ> ransac_fit_line(pcl::PointCloud<pcl::PointXYZ> edges, Eigen::VectorXf& model_coefficients)
{
	std::vector<pcl::PointXYZ> points_set = cloudToArray(edges);
	pcl::PointCloud<pcl::PointXYZ> inlier_points;

	if(points_set.size()<40){inlier_points.clear();return inlier_points;}

	std::vector<int> inliers_idx;
	Ransac_Line Line;
	Line.setInputCloud(points_set);
	Line.setDistanceThreshold(0.3);
	Line.computeModel();
	Line.getInliers(inliers_idx);

	Line.getModelCoefficients(model_coefficients);


	for(int i=0; i<inliers_idx.size(); i++)
		inlier_points.push_back(points_set[inliers_idx[i]]);

	return	inlier_points;
}

template <typename PointT>
void test_template(PointT p)
{
	cout<<"x: "<<p.x<<"\t"<<"y: "<<p.y<<"\t"<<"z: "<<p.z<<endl;
}


}//namespace filter





}//namespace cloud_operation


#endif
