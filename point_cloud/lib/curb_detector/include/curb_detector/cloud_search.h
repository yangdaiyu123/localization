#ifndef _CLOUD_SEARCH_H_
#define	_CLOUD_SEARCH_H_

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

template <typename PointT>
pcl::PointCloud<PointT> getOneRing(const vector<pcl::PointCloud<PointT> > cloud_array, const int ring_id)
{
	pcl::PointCloud<PointT> cloud;
	if(ring_id>=cloud_array.size())	{std::cout<<"ring_id out of array size\n"; return cloud;}

	for(int i=0; i<cloud_array[ring_id].size(); i++)
		cloud.push_back(cloud_array[ring_id].points[i]);
	return cloud;
}

template <typename PointT>
pcl::PointCloud<PointT> getOneRingSection(const vector<pcl::PointCloud<PointT> > cloud_sorted,
												const int ring_id, const int start_idx, const int end_idx)
{
	pcl::PointCloud<PointT> cloud;
	if(ring_id>=cloud_sorted.size())	{std::cout<<"ring_id out of array size\n"; return cloud;}
//	if(start_idx<0 )

	for(int i=start_idx; i<end_idx; i++)
		cloud.push_back(cloud_sorted[ring_id].points[i]);
	return cloud;
}

template <typename PointT>
pcl::PointCloud<PointT> searchSectionEdgePoint(pcl::PointCloud<PointT> cloud,
														int start, int end, int step, int window_size,int ratio_th)
{
	pcl::PointCloud<PointT> edge_points;
//	cout<<"searchSectionEdgePoint"<<endl;
	if(start==-1 || end==-1)
	{
		setPointsInvalid(cloud);
		return edge_points;
	}

	if(end>start)
	{
//		cout<<"end>start"<<endl;
		for(int i=start; i<end; i+=abs(step))
		{
			double r = getDevRatio(cloud,i,i+window_size);
			if((int)r > ratio_th)
			{
				for(int mark_id=i; mark_id<(i+window_size); mark_id++)
					edge_points.push_back(cloud.points[mark_id]);
				break;
			}
		}
//		cout<<"end>start"<<endl;
	}
	else
	{
//		cout<<"end<start"<<endl;
//		cout<<"start"<<start<<"end"<<end<<endl;
		for(int i=start; i>end; i-=abs(step))
		{
			double ratio = getDevRatio(cloud,i-window_size,i);
			if(ratio > ratio_th)
			{
				for(int mark_id=i; mark_id>(i-window_size); mark_id--)
					edge_points.push_back(cloud.points[mark_id]);
				break;
			}
		}
//		cout<<"end<start"<<endl;
	}
	if(edge_points.empty())
		setPointsInvalid(cloud);

//	cout<<"searchSectionEdgePoint"<<endl;

	return edge_points;
}

template <typename PointT>
pcl::PointCloud<PointT> searchSectionSignPoint(pcl::PointCloud<PointT> cloud,
														int start, int end, int step, int window_size,int ratio_th)
{
	pcl::PointCloud<PointT> sign_points;
	if(start==-1 || end==-1)
	{
		setPointsInvalid(cloud);
		return sign_points;
	}

//	cout<<"searchSectionSignPoint_in"<<endl;
	for(int i=start; i<end; i+=abs(step))
	{
//		double r = getDevRatio(cloud,i,i+window_size);
		double dev_x,dev_y,dev_z;
		getDevXYZ(cloud, i, i+window_size,dev_x,dev_y,dev_z);
		double ratio = dev_y / (dev_x * dev_z);
		double r = getDevRatio(cloud, i, i+window_size, 1);
		cout<<"ratio "<<ratio<<endl;
		if((int)ratio > ratio_th && (int)dev_x < 5 && (int)dev_z < 5)
		{
			for(int mark_id=i; mark_id<(i+window_size); mark_id++)
				sign_points.push_back(cloud.points[mark_id]);
//			break;
		}
	}


	if(sign_points.empty())
		setPointsInvalid(cloud);
//	cout<<"searchSectionSignPoint_out"<<endl;
	return sign_points;
}


template <typename PointT>
vector<pcl::PointCloud<PointT> > searchHeadRight(const vector<pcl::PointCloud<PointT> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchHeadRight"<<endl;
	vector<pcl::PointCloud<PointT> > mark_points;
	int ring_total = cloud_sorted.size();
	mark_points.resize(ring_total);

	for(int ring_id=0; ring_id < ring_total; ring_id++)
	{
		pcl::PointCloud<pcl::PointXYZI> edge_points;

		if(cloud_sorted[ring_id].points.size() < 20)
		{
			setPointsInvalid(edge_points);
			mark_points.push_back(edge_points);
//			return	mark_points;
			continue;
		}
		int start_idx=2*window_size;
		int end_idx=cloud_sorted[ring_id].size()-2*window_size;


//		cout<<"searchHeadRight\t"<<"ring_id: "<<ring_id<<"\tend_idx: "<<cloud_sorted[ring_id].size()<<endl;
		edge_points = searchSectionEdgePoint(cloud_sorted[ring_id],start_idx, end_idx, step, window_size, ratio_th);
		for (typename pcl::PointCloud<PointT>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
		{
		    if ((*it).x < 0) it = edge_points.erase(it);
		    else ++it;
		}
		if(edge_points.empty())	setPointsInvalid(edge_points);
		mark_points.push_back(edge_points);
	}

//	cout<<"searchHeadRight"<<endl;

	return	mark_points;
}

template <typename PointT>
vector<pcl::PointCloud<PointT> > searchHeadLeft(const vector<pcl::PointCloud<PointT> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchHeadLeft"<<endl;
	vector<pcl::PointCloud<PointT> > mark_points;
	int ring_total = cloud_sorted.size();
	mark_points.resize(ring_total);

	for(int ring_id=0; ring_id < ring_total; ring_id++)
	{
		pcl::PointCloud<pcl::PointXYZI> edge_points;
		if(cloud_sorted[ring_id].points.size() < 20)
		{
			setPointsInvalid(edge_points);
			mark_points.push_back(edge_points);
//			return	mark_points;
			continue;
		}

		int start_idx=cloud_sorted[ring_id].size()-2*window_size;
		int end_idx=2*window_size;

//		cout<<"searchHeadLeft"<<endl;
		edge_points = searchSectionEdgePoint(cloud_sorted[ring_id],start_idx, end_idx, step, window_size, ratio_th);
		for (typename pcl::PointCloud<PointT>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
		{
		    if ((*it).x < 0) it = edge_points.erase(it);
		    else ++it;
		}
		if(edge_points.empty())	setPointsInvalid(edge_points);
		mark_points.push_back(edge_points);
	}

//	cout<<"searchHeadLeft"<<endl;
	return	mark_points;
}

template <typename PointT>
vector<pcl::PointCloud<PointT> > searchRearRight(const vector<pcl::PointCloud<PointT> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchRearRight"<<endl;
	vector<pcl::PointCloud<PointT> > mark_points;
	int ring_total = cloud_sorted.size();
	mark_points.resize(ring_total);

	for(int ring_id=0; ring_id < ring_total; ring_id++)
	{
		pcl::PointCloud<pcl::PointXYZI> edge_points;
		if(cloud_sorted[ring_id].points.size() < 20)
		{
			setPointsInvalid(edge_points);
			mark_points.push_back(edge_points);
			return	mark_points;
		}

		int start_idx=cloud_sorted[ring_id].size()-2*window_size;
		int end_idx=2*window_size;

//		cout<<"searchRearRight"<<endl;
		edge_points = searchSectionEdgePoint(cloud_sorted[ring_id],start_idx, end_idx, step, window_size, ratio_th);
		for (typename pcl::PointCloud<PointT>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
		{
		    if ((*it).x > 0 || abs((*it).y)<1 ) it = edge_points.erase(it);
		    else ++it;
		}
		if(edge_points.empty())	setPointsInvalid(edge_points);
		mark_points.push_back(edge_points);
	}
//	cout<<"searchRearRight"<<endl;
	return	mark_points;
}

template <typename PointT>
vector<pcl::PointCloud<PointT> > searchRearLeft(const vector<pcl::PointCloud<PointT> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchRearLeft"<<endl;
	vector<pcl::PointCloud<PointT> > mark_points;
	int ring_total = cloud_sorted.size();
	mark_points.resize(ring_total);

	for(int ring_id=0; ring_id < ring_total; ring_id++)
	{
		pcl::PointCloud<pcl::PointXYZI> edge_points;
		if(cloud_sorted[ring_id].points.size() < 20)
		{
			setPointsInvalid(edge_points);
			mark_points.push_back(edge_points);
			return	mark_points;
		}

		int start_idx=2*window_size;
		int end_idx=cloud_sorted[ring_id].size()-2*window_size;

//		cout<<"searchRearLeft"<<endl;
		edge_points = searchSectionEdgePoint(cloud_sorted[ring_id],start_idx, end_idx, step, window_size, ratio_th);
		for (typename pcl::PointCloud<PointT>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
		{
		    if ((*it).x > 0 || abs((*it).y)<1) it = edge_points.erase(it);
		    else ++it;
		}
		if(edge_points.empty())	setPointsInvalid(edge_points);
		mark_points.push_back(edge_points);
	}
//	cout<<"searchRearLeft"<<endl;
	return	mark_points;
}

template <typename PointT>
vector<pcl::PointCloud<PointT> > searchSignPoints(const vector<pcl::PointCloud<PointT> > cloud_sorted,
														const int step, const int window_size,const int ratio_th)
{
	vector<pcl::PointCloud<PointT> > mark_points;
	int ring_total = cloud_sorted.size();
	mark_points.resize(ring_total);
//	cout<<"ring_total "<<ring_total<<endl;
//	cout<<"searchSignPoints"<<endl;
	for(int ring_id=0; ring_id < ring_total; ring_id++)
	{
		pcl::PointCloud<pcl::PointXYZI> sign_points;
//		cout<<"ring_id "<<ring_id<<"\tsize "<<cloud_sorted[ring_id].points.size()<<endl;
		if(cloud_sorted[ring_id].points.size() < 50)
		{
			setPointsInvalid(sign_points);
			mark_points.push_back(sign_points);
//			return	mark_points;
		}

		int start_idx=2*window_size;
		int end_idx=cloud_sorted[ring_id].size()-2*window_size;
//		cout<<"searchSectionSignPoint"<<endl;
//		edge_points = searchSectionEdgePoint(cloud_sorted[ring_id],start_idx, end_idx, step, window_size, ratio_th);
		sign_points = searchSectionSignPoint(cloud_sorted[ring_id],start_idx, end_idx, step, window_size, ratio_th);
		if(sign_points.empty())	setPointsInvalid(sign_points);
		mark_points.push_back(sign_points);
	}
	return	mark_points;
}

}//namespace cloud_operation


#endif
