#ifndef _CLOUD_OPERATION_H
#define	_CLOUD_OPERATION_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>

#include <point_types.h>
#include <ransac_line/ransac_line.h>

#include <string>
#include <iostream>
#include <fstream>



namespace cloud_operation
{

using namespace std;
using namespace pcl;

const int	INVAILD_LABEL=	-100;

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

void getHeadRearIdx(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array,
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

vector<pcl::PointCloud<pcl::PointXYZI> > sortCloudArray(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array,
														const vector<int> head_idx, const vector<int> rear_idx)
{
	int ring_total = cloud_array.size();
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted;
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

vector<pcl::PointCloud<pcl::PointXYZI> > sortCloudArrayReverse(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array,
														const vector<int> head_idx, const vector<int> rear_idx)
{
	int ring_total = cloud_array.size();
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted;
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

pcl::PointCloud<pcl::PointXYZI> getOneRing(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array, const int ring_id)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	if(ring_id>=cloud_array.size())	{std::cout<<"ring_id out of array size\n"; return cloud;}

	for(int i=0; i<cloud_array[ring_id].size(); i++)
		cloud.push_back(cloud_array[ring_id].points[i]);
	return cloud;
}

pcl::PointCloud<pcl::PointXYZI> getOneRingSection(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted,
												const int ring_id, const int start_idx, const int end_idx)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	if(ring_id>=cloud_sorted.size())	{std::cout<<"ring_id out of array size\n"; return cloud;}
//	if(start_idx<0 )

	for(int i=start_idx; i<end_idx; i++)
		cloud.push_back(cloud_sorted[ring_id].points[i]);
	return cloud;
}

double getDevX(pcl::PointCloud<pcl::PointXYZI> cloud, int start, int end)
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

double getDevY(pcl::PointCloud<pcl::PointXYZI> cloud, int start, int end)
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

double getDevXYZ(pcl::PointCloud<pcl::PointXYZI> cloud, int start, int end, double& dev_x, double& dev_y, double& dev_z)
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
//	if(ratio>100)
//	{
//		cout<<"sum_x: "<<sum_x<<"\tmean_x: "<<mean_x<<"\tsum_dx2: "<<sum_dx2<<endl;
//		cout<<"sum_y: "<<sum_y<<"\tmean_y: "<<mean_y<<"\tsum_dy2: "<<sum_dy2<<endl;
//		cout<<"sum_z: "<<sum_z<<"\tmean_z: "<<mean_z<<"\tsum_dz2: "<<sum_dz2<<endl;

//	}


	return	sum_dy2;
}

double getDevRatio(pcl::PointCloud<pcl::PointXYZI> cloud, int start, int end)
{
	double dev_x = getDevX(cloud, start, end);
	double dev_y = getDevY(cloud, start, end);
	double ratio = dev_x/(dev_y+0.0001);
//	cout<<"x: "<<dev_x<<"\ty: "<<dev_y<<"\tratio: "<<ratio<<endl;
//	cout<<endl;
	return ratio;
}

double getDevRatio(pcl::PointCloud<pcl::PointXYZI> cloud, int start, int end, bool swap_x_y)
{
	double dev_x = getDevX(cloud, start, end);
	double dev_y = getDevY(cloud, start, end);
	double ratio = 0;
	if(swap_x_y)	ratio = dev_y/(dev_x+0.0001);
	else	ratio = dev_x/(dev_y+0.0001);
	return ratio;
}

void setPointsInvalid(pcl::PointCloud<pcl::PointXYZI>& cloud)
{
	pcl::PointXYZI p;
	p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=0;
	cloud.clear();
	cloud.push_back(p);
}

bool isPointValid(const pcl::PointXYZI p)
{

}

pcl::PointCloud<pcl::PointXYZI> searchSectionEdgePoint(pcl::PointCloud<pcl::PointXYZI> cloud,
														int start, int end, int step, int window_size,int ratio_th)
{
	pcl::PointCloud<pcl::PointXYZI> edge_points;
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

pcl::PointCloud<pcl::PointXYZI> searchSectionSignPoint(pcl::PointCloud<pcl::PointXYZI> cloud,
														int start, int end, int step, int window_size,int ratio_th)
{
	pcl::PointCloud<pcl::PointXYZI> sign_points;
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

vector<pcl::PointCloud<pcl::PointXYZI> > searchHeadRight(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchHeadRight"<<endl;
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
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
		for (pcl::PointCloud<PointXYZI>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
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

vector<pcl::PointCloud<pcl::PointXYZI> > searchHeadLeft(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchHeadLeft"<<endl;
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
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
		for (pcl::PointCloud<PointXYZI>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
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

vector<pcl::PointCloud<pcl::PointXYZI> > searchRearRight(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchRearRight"<<endl;
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
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
		for (pcl::PointCloud<PointXYZI>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
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

vector<pcl::PointCloud<pcl::PointXYZI> > searchRearLeft(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted,
									const int step, const int window_size,const int ratio_th)
{
//	cout<<"searchRearLeft"<<endl;
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
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
		for (pcl::PointCloud<PointXYZI>::iterator it = edge_points.begin(); it != edge_points.end(); /**/)
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

vector<pcl::PointCloud<pcl::PointXYZI> > searchSignPoints(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted,
														const int step, const int window_size,const int ratio_th)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
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

pcl::PointCloud<pcl::PointXYZI> arrayToCloudXYZI(const vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::PointXYZI p;
	for(int ring_id=0; ring_id<cloud_array.size(); ring_id++)
	{
		for(int point_id=0; point_id < cloud_array[ring_id].size(); point_id++)
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
		for(int point_id=0; point_id < cloud_array[ring_id].size(); point_id++)
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
		for(int point_id=0; point_id < cloud_array[ring_id].size(); point_id++)
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

void markPoints(pcl::PointCloud<pcl::PointXYZRGB>& color_cloud, vector<pcl::PointCloud<pcl::PointXYZI> > mark_points, int mark_color)
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

void markPoints(pcl::PointCloud<pcl::PointXYZRGB>& color_cloud, pcl::PointCloud<pcl::PointXYZ>  mark_points, int mark_color)
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

void markPoints(pcl::PointCloud<pcl::PointXYZRGB>& color_cloud, pcl::PointCloud<pcl::PointXYZI>  mark_points, int mark_color)
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

namespace filter
{


bool is_point_available(pcl::PointXYZRGB p)
{
	if(	int(p.z)==INVAILD_LABEL)
		return	false;
	else	return true;
}

bool is_point_available(pcl::PointXYZ p)
{
	if(	int(p.z)==INVAILD_LABEL)
		return	false;
	else	return true;
}

bool is_point_available(pcl::PointXYZI p)
{
	if(	int(p.z)==INVAILD_LABEL)
		return	false;
	else	return true;
}

void remove_invalid_points(pcl::PointCloud<pcl::PointXYZ>&  cloud)
{
	for (pcl::PointCloud<PointXYZ>::iterator it = cloud.begin(); it != cloud.end(); /**/)
	    if ( !is_point_available(*it) ) it = cloud.erase(it);
	    else ++it;
}

void remove_invalid_points(pcl::PointCloud<pcl::PointXYZRGB>&  cloud)
{
	for (pcl::PointCloud<PointXYZRGB>::iterator it = cloud.begin(); it != cloud.end(); /**/)
	    if ( !is_point_available(*it) ) it = cloud.erase(it);
	    else ++it;
}

bool is_position_reasonable(pcl::PointXYZ p)
{
	if(p.z>0.5)	return false;
	else if(abs(p.y)>8) return false;
	else return true;
}

void dist_filter(pcl::PointCloud<pcl::PointXYZ>&  cloud)
{
	for (pcl::PointCloud<PointXYZ>::iterator it = cloud.begin(); it != cloud.end(); /**/)
		if ( !is_position_reasonable(*it) ) it = cloud.erase(it);
	    else ++it;
}

std::vector<pcl::PointXYZ> cloudToArray(const pcl::PointCloud<pcl::PointXYZ> cloud)
{
	std::vector<pcl::PointXYZ> points_set;
	for(int i=0; i<cloud.size(); i++)
		points_set.push_back(cloud.points[i]);
	return points_set;
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


}//namespace filter





}//namespace cloud_operation


#endif
