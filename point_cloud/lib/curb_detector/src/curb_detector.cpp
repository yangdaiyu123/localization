#include "curb_detector.h"

void CurbDetector::setInputCloud(const VPointCloud::ConstPtr &inMsg)
{
	cloud_in_ = inMsg;
}

vector<pcl::PointCloud<pcl::PointXYZI> > CurbDetector::sortCloud()
{
//	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array;
//	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted;
//	cloud_array.resize(ARRAY_SIZE);
//	cloud_sorted.resize(ARRAY_SIZE);

//	for (int i = 1; i < inMsg->points.size(); i++)
//	{
//		if(inMsg->points[i].ring < ARRAY_SIZE)
//		{
//			int ring_idx = inMsg->points[i].ring;
//			pcl::PointXYZI	pt;
//			pt.x = inMsg->points[i].x;
//			pt.y = inMsg->points[i].y;
//			pt.z = inMsg->points[i].z;
//			pt.intensity = inMsg->points[i].intensity;

////			if(dist_filter(pt,1.5))	continue;

////			cloud_array[ring_idx].push_back(pt);

////			if( abs(pt.x)<0.2)
////			{
////				 if(pt.y>0)	head_idx[ring_idx] = cloud_array[ring_idx].size();
////			}

//		}
//	}
//	for(int ring_idx=0; ring_idx<ARRAY_SIZE; ring_idx++)
//		cout<<"ring_idx"<<ring_idx<<"\t"<<head_idx[ring_idx]<<endl;

//	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
//	{

//		for(int i=head_idx[cloud_id]; i<cloud_array[cloud_id].size(); i++)
//		{
////			cout<<"no problem"<<endl;
//			cloud_sorted[cloud_id].push_back(cloud_array[cloud_id].points[i]);
//		}


//		for(int i=0; i<=head_idx[cloud_id]; i++)
//			cloud_sorted[cloud_id].push_back(cloud_array[cloud_id].points[i]);

//	}

//	return	cloud_sorted;
}
