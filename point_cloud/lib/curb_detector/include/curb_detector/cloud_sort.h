#ifndef _CLOUD_SORT_H_
#define	_CLOUD_SORT_H_

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
void getHeadRearIdx(const vector<pcl::PointCloud<PointT> > cloud_array,
					vector<int>& head_idx, vector<int>& rear_idx)
{
	int ring_total = cloud_array.size();
	head_idx.resize(ring_total);
	rear_idx.resize(ring_total);

	for(int ring_id=0; ring_id < ring_total; ring_id++)
	{
		bool has_rear_idx = false;
		bool has_head_idx = false;

		for(int i=1; i<cloud_array[ring_id].size(); i+=3)
		{
			pcl::PointXYZI	pt;
			pt.x = cloud_array[ring_id].points[i].x;
			pt.y = cloud_array[ring_id].points[i].y;

			if( abs(pt.y)<2)
			{
				if(pt.x<0)
				{
					rear_idx[ring_id] = i;
					has_rear_idx = true;
				}
				else
				{
					head_idx[ring_id] = i;
					has_head_idx = true;
				}
			}
			if(has_head_idx && has_rear_idx)	break;
		}
		if(head_idx[ring_id]==0)	head_idx[ring_id] = -1;
		if(rear_idx[ring_id]==0)	rear_idx[ring_id] = -1;
	}

//	for(int i=0; i<ring_total; i++)
//		cout<<"ring: "<<i<<"\tsize: "<<cloud_array[i].size()<<"\thead_idx: "<<head_idx[i]<<"\trear_idx: "<<rear_idx[i]<<endl;
}

template <typename PointT>
vector<pcl::PointCloud<PointT> > sortCloudArray(const vector<pcl::PointCloud<PointT> > cloud_array,
														const vector<int> head_idx, const vector<int> rear_idx)
{
	int ring_total = cloud_array.size();
	vector<pcl::PointCloud<PointT> > cloud_sorted;
	cloud_sorted.resize(ring_total);

	for(int ring_id=0; ring_id<ring_total; ring_id++)
	{
		for(int i=head_idx[ring_id]; i<cloud_array[ring_id].size(); i++)
			cloud_sorted[ring_id].push_back(cloud_array[ring_id].points[i]);

		for(int i=0; i<=head_idx[ring_id]; i++)
			cloud_sorted[ring_id].push_back(cloud_array[ring_id].points[i]);
	}
	return	cloud_sorted;

}

template <typename PointT>
vector<pcl::PointCloud<PointT> > sortCloudArrayReverse(const vector<pcl::PointCloud<PointT> > cloud_array,
														const vector<int> head_idx, const vector<int> rear_idx)
{
	int ring_total = cloud_array.size();
	vector<pcl::PointCloud<PointT> > cloud_sorted;
	cloud_sorted.resize(ring_total);

	for(int ring_id=0; ring_id<ring_total; ring_id++)
	{
		for(int i=rear_idx[ring_id]; i<cloud_array[ring_id].size(); i++)
			cloud_sorted[ring_id].push_back(cloud_array[ring_id].points[i]);

		for(int i=0; i<=rear_idx[ring_id]; i++)
			cloud_sorted[ring_id].push_back(cloud_array[ring_id].points[i]);
	}
	return	cloud_sorted;
}

}//namespace cloud_operation

#endif
