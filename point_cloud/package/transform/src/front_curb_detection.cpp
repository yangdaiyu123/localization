#include "curb_detection.h"
#include <fstream>

using namespace std;
using namespace pcl;

#define DEBUG_MODE 1

#if DEBUG_MODE
pcl::visualization::CloudViewer MyViewer("cloud");
#endif

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ring(new pcl::PointCloud<pcl::PointXYZRGB>);


CurbDetection::CurbDetection()
{
//------成员变量初始化-------------	
	yaw_vel_cur = 0;
	yaw_vel_last = 0;
	velocity_cur = 0;
	velocity_last = 0;
	time_cur = ros::Time::now().toSec();
	time_last = time_cur;
	first_time = true;
	received_velocity = false;
	received_yaw_vel = false;
	for(int i=0; i<ARRAY_SIZE; i++)	head_idx[i]=0;
	for(int i=0; i<ARRAY_SIZE; i++)	rear_idx[i]=0;
	
	cloud_new = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_sigma = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	sub_point_cloud = nh.subscribe("points_in_car_coord", 2, &CurbDetection::pointCloudCallback,this);
	sub_velocity = nh.subscribe("velocity",2,&CurbDetection::velocityCallback,this);
	sub_imu = nh.subscribe("/imu_torso/xsens/data",2,&CurbDetection::imuCallback,this);
	pub_left_edges = nh.advertise<sensor_msgs::PointCloud2>("left_edge_cloud_raw",1);
	pub_right_edges = nh.advertise<sensor_msgs::PointCloud2>("right_edge_cloud_raw",1);
#if PUB_CLOUD_RING
	pub_cloud_ring = nh.advertise<sensor_msgs::PointCloud2>("color_cloud",1);
#endif
//	ros::spin();
	ros::MultiThreadedSpinner spinner(2); // Use 2 threads
	spinner.spin(); // spin() will not return until the node has been shutdown
}

CurbDetection::~CurbDetection()
{
	
}

void CurbDetection::velocityCallback(const std_msgs::Float64MultiArray::ConstPtr& v_in)
{
//	cout<<"velocity: "<<v_in->data[0]<<endl;
	velocity_cur = v_in->data[0];
	received_velocity = true;
}

void CurbDetection::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
//	cout<<imu_msg->angular_velocity.z<<endl;
	yaw_vel_cur = imu_msg->angular_velocity.z;
	received_yaw_vel = true;
}


//接收点云的回调函数，所有处理从这里开始
void CurbDetection::pointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
	if(received_velocity==false || received_yaw_vel==false)
		return;
	
	cloud_ring->points.clear();	
	
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array = sort_point_clouds(inMsg);
	
	get_rear_idx(cloud_array, rear_idx);
	
	vector<pcl::PointCloud<pcl::PointXYZI> > edge_head_right = search_head_right(cloud_array);
	vector<pcl::PointCloud<pcl::PointXYZI> > edge_head_left = search_head_left(cloud_array);
	vector<pcl::PointCloud<pcl::PointXYZI> > edge_rear_right = search_rear_right(cloud_array);
	vector<pcl::PointCloud<pcl::PointXYZI> > edge_rear_left = search_rear_left(cloud_array);	
	
		
	
	right_edge_cloud = add_edges_cloud(edge_head_right,edge_rear_right);
	left_edge_cloud = add_edges_cloud(edge_head_left,edge_rear_left);
	double t0 = ros::Time::now().toSec();
	splice_edges();
	double t1 = ros::Time::now().toSec();
	cout<<"duration: "<<t1-t0<<endl;
	
	sensor_msgs::PointCloud2 left_cloud_to_pub;
    pcl::toROSMsg(left_edge_sigma, left_cloud_to_pub);
    pub_left_edges.publish(left_cloud_to_pub);
    
    sensor_msgs::PointCloud2 right_cloud_to_pub;
    pcl::toROSMsg(right_edge_sigma, right_cloud_to_pub);
    pub_right_edges.publish(right_cloud_to_pub);
   
#if DEBUG_MODE 
    visualize(cloud_array,edge_head_right,edge_head_left,edge_rear_right,edge_rear_left);
	MyViewer.showCloud(cloud_ring,"cloud");
#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curb_detection");
	
	CurbDetection CurbDetection_obj;
	
	return 0;
}


/*
将点云按环分类，类数由宏定义ARRAY_SIZE决定
每一环点云以y方向正前方为起始，顺时针生长
*/
vector<pcl::PointCloud<pcl::PointXYZI> > CurbDetection::sort_point_clouds(const VPointCloud::ConstPtr &inMsg)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array;
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted;
	cloud_array.resize(ARRAY_SIZE);
	cloud_sorted.resize(ARRAY_SIZE);
	
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
			
			if(dist_filter(pt,1.2))	continue;
			
			cloud_array[ring_idx].push_back(pt);
			
			if( abs(pt.x)<0.2)
			{
				 if(pt.y>0)	head_idx[ring_idx] = cloud_array[ring_idx].size();
//				 if(pt.x<0)	rear_idx[ring_idx] = cloud_array[ring_idx].size();
			}
			
		}
	}
	for(int ring_idx=0; ring_idx<ARRAY_SIZE; ring_idx++)
		cout<<"ring_idx"<<ring_idx<<"\t"<<head_idx[ring_idx]<<endl;
	
	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		
		for(int i=head_idx[cloud_id]; i<cloud_array[cloud_id].size(); i++)
		{
//			cout<<"no problem"<<endl;
			cloud_sorted[cloud_id].push_back(cloud_array[cloud_id].points[i]);
		}
			
		
		for(int i=0; i<=head_idx[cloud_id]; i++)
			cloud_sorted[cloud_id].push_back(cloud_array[cloud_id].points[i]);
		
	}

	return	cloud_sorted;
}


/*
将历史路沿点转换并叠加到当前坐标系下的路沿点
*/
void CurbDetection::splice_edges()
{
	time_cur = ros::Time::now().toSec();
		
	if(!first_time)
	{
		time_incr = (time_cur - time_last);
		yaw_incr = (yaw_vel_cur + yaw_vel_last)*time_incr / 2;
		odometry_incr = (velocity_cur + velocity_last) / 2 / 3.6*time_incr;

		Eigen::Affine3f transform_a = Eigen::Affine3f::Identity();
		transform_a.translation() << -odometry_incr*sin(yaw_incr), -odometry_incr*cos(yaw_incr), 0.0;
		transform_a.rotate(Eigen::AngleAxisf(-yaw_incr, Eigen::Vector3f::UnitZ()));
		
		pcl::transformPointCloud(left_edge_sigma, left_edge_sigma, transform_a);
		pcl::transformPointCloud(right_edge_sigma, right_edge_sigma, transform_a);
	}

	first_time = false;
	time_last = time_cur;
	velocity_last = velocity_cur;
	yaw_vel_last = yaw_vel_cur;
	
	cloud_size_left.push_back(left_edge_cloud.size());
	left_edge_sigma = left_edge_sigma + left_edge_cloud;	
	if (cloud_size_left.size()>10)
	{
		left_edge_sigma.erase(left_edge_sigma.begin(), left_edge_sigma.begin() + cloud_size_left[0]);
		cloud_size_left.erase(cloud_size_left.begin());
	}
	
	cloud_size_right.push_back(right_edge_cloud.size());
	right_edge_sigma = right_edge_sigma + right_edge_cloud;
	if(cloud_size_right.size()>10)
	{
		right_edge_sigma.erase(right_edge_sigma.begin(), right_edge_sigma.begin() + cloud_size_right[0]);
		cloud_size_right.erase(cloud_size_right.begin());
	}
}

/*
获取正后方点云的索引
*/
void CurbDetection::get_rear_idx(vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array, int* rear_idx)
{
	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		int start_idx=0;
		int end_idx=cloud_array[cloud_id].size();
		int step=3;
		
		for(int i=0; i<cloud_array[cloud_id].size(); i+=step)
		{
			pcl::PointXYZI	pt;
			pt.x = cloud_array[cloud_id].points[i].x;
			pt.y = cloud_array[cloud_id].points[i].y;
			if( abs(pt.x)<0.2 && pt.y<0)
			{
				rear_idx[cloud_id] = i;
#ifndef DEBUG_MODE
				cout<<"rear_idx\t"<<i<<endl;
				cout<<"size\t"<<cloud_array[cloud_id]<<endl;
#endif
			}			
		}
	}
}

/*
搜索右前方区间的路沿,由于每个区间起始点和搜索方向不同，故分为四个搜索区间
*/
vector<pcl::PointCloud<pcl::PointXYZI> > CurbDetection::search_head_right(vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
	mark_points.resize(ARRAY_SIZE);
	
	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		int start_idx=0;
		int end_idx=rear_idx[cloud_id];
		int step=3;
		ofstream ofile;
    	ofile.open("/home/cyberc3/ratio.txt",ios::app);
		
		for(int i=start_idx; i<end_idx; i+=step)
		{
			double dev_x = get_dev_x(cloud_array[cloud_id], i, WINDOW_SIZE);
			double dev_y = get_dev_y(cloud_array[cloud_id], i, WINDOW_SIZE);
			double ratio = dev_y/dev_x;
			
			ofile<<ratio<<endl;

			if(ratio>EDGE_THRESHOLD)
			{
				if( slope_filter(cloud_array[cloud_id].points[i] , SLOPE_TH) )
				{
					pcl::PointXYZI p;
					p.x=INVAILD_LABEL;
					p.y=INVAILD_LABEL;
					p.z=INVAILD_LABEL;
					mark_points[cloud_id].push_back(p);
					break;
				}
				else
				{
					mark_points[cloud_id].push_back(cloud_array[cloud_id].points[i]);
					for(int mark_id=(i+1); mark_id<(i+WINDOW_SIZE); mark_id++)
					{
						mark_points[cloud_id].push_back(cloud_array[cloud_id].points[mark_id]);
					}
					break;
				}
			}
		}
		if(mark_points[cloud_id].empty())
		{
			pcl::PointXYZI p;
			p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
			mark_points[cloud_id].push_back(p);
		}
		ofile<<"ring end"<<endl;
		ofile.close();
	}
	
	return	mark_points;
}

/*
搜索左前方区间的路沿,由于每个区间起始点和搜索方向不同，故分为四个搜索区间
*/
vector<pcl::PointCloud<pcl::PointXYZI> > CurbDetection::search_head_left(vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
	mark_points.resize(ARRAY_SIZE);
	
	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		int start_idx=cloud_array[cloud_id].size()-WINDOW_SIZE;
		int end_idx=rear_idx[cloud_id];
		int step=3;
		
		for(int i=start_idx; i>end_idx; i-=step)
		{
			double dev_x = get_dev_x(cloud_array[cloud_id], i, -WINDOW_SIZE);
			double dev_y = get_dev_y(cloud_array[cloud_id], i, -WINDOW_SIZE);
			double ratio = dev_y/dev_x;

			if(ratio>EDGE_THRESHOLD)
			{
				if( slope_filter(cloud_array[cloud_id].points[i] , SLOPE_TH) )
				{
					pcl::PointXYZI p;
					p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
					mark_points[cloud_id].push_back(p);
					break;
				}
				else
				{
					mark_points[cloud_id].push_back(cloud_array[cloud_id].points[i]);
					for(int mark_id=(i-WINDOW_SIZE); mark_id<(i-1); mark_id++)
					{
						mark_points[cloud_id].push_back(cloud_array[cloud_id].points[mark_id]);
					}
					break;
				}
			}
		}
		if(mark_points[cloud_id].empty())
		{
			pcl::PointXYZI p;
			p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
			mark_points[cloud_id].push_back(p);
		}
	}
	return	mark_points;
}

/*
搜索右后方区间的路沿
*/
vector<pcl::PointCloud<pcl::PointXYZI> > CurbDetection::search_rear_right(vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
	mark_points.resize(ARRAY_SIZE);
	
	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		int start_idx=rear_idx[cloud_id];
		int end_idx=0;
		int step=3;
		
		for(int i=start_idx; i>end_idx; i-=step)
		{
			double dev_x = get_dev_x(cloud_array[cloud_id], i, -WINDOW_SIZE);
			double dev_y = get_dev_y(cloud_array[cloud_id], i, -WINDOW_SIZE);
			double ratio = dev_y/dev_x;
			
			if(ratio>EDGE_THRESHOLD)
			{
				if( slope_filter(cloud_array[cloud_id].points[i] , SLOPE_TH) )
				{
					pcl::PointXYZI p;
					p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
					mark_points[cloud_id].push_back(p);
					break;
				}
				else
				{
					mark_points[cloud_id].push_back(cloud_array[cloud_id].points[i]);
					for(int mark_id=(i-WINDOW_SIZE); mark_id<(i-1); mark_id++)
					{
						mark_points[cloud_id].push_back(cloud_array[cloud_id].points[mark_id]);
					}
					break;
				}
			}
		}
		if(mark_points[cloud_id].empty())
		{
			pcl::PointXYZI p;
			p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
			mark_points[cloud_id].push_back(p);
		}
	}
	return	mark_points;	
}
/*
搜索左后方区间的路沿
*/
vector<pcl::PointCloud<pcl::PointXYZI> > CurbDetection::search_rear_left(vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array)
{	
	vector<pcl::PointCloud<pcl::PointXYZI> > mark_points;
	mark_points.resize(ARRAY_SIZE);

	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		int start_idx=rear_idx[cloud_id];
		int end_idx=cloud_array[cloud_id].size();
		int step=3;
		
		for(int i=start_idx; i<end_idx; i+=step)
		{
			double dev_x = get_dev_x(cloud_array[cloud_id], i, WINDOW_SIZE);
			double dev_y = get_dev_y(cloud_array[cloud_id], i, WINDOW_SIZE);
			double ratio = dev_y/dev_x;
	
			if(ratio>EDGE_THRESHOLD)
			{
				if( slope_filter(cloud_array[cloud_id].points[i] , SLOPE_TH) )
				{
					pcl::PointXYZI p;
					p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
					mark_points[cloud_id].push_back(p);
					break;
				}
				else
				{
					mark_points[cloud_id].push_back(cloud_array[cloud_id].points[i]);
					for(int mark_id=(i+1); mark_id<(i+WINDOW_SIZE); mark_id++)
					{
						mark_points[cloud_id].push_back(cloud_array[cloud_id].points[mark_id]);
					}
					break;
				}
			}
		}
		if(mark_points[cloud_id].empty())
		{
			pcl::PointXYZI p;
			p.x=INVAILD_LABEL;p.y=INVAILD_LABEL;p.z=INVAILD_LABEL;p.intensity=2;
			mark_points[cloud_id].push_back(p);
		}
	}
	return	mark_points;	
}

/*
将前方路沿和后方路沿合并
edge_array不仅存放了每一环第1个检测到的路沿跳变点
为了便于显示，还额外存放了后面5个检测到的路沿跳变点
这些都是检测出的路沿跳变点，也可用于后面直线拟合，这里只取第一个
*/
pcl::PointCloud<pcl::PointXYZI> CurbDetection::add_edges_cloud(	vector<pcl::PointCloud<pcl::PointXYZI> > edge_array1, 
										vector<pcl::PointCloud<pcl::PointXYZI> > edge_array2)
{
	pcl::PointCloud<pcl::PointXYZI> edges_cloud;
	if(!edge_array1.empty())
		for(int i=0; i<edge_array1.size(); i++)
		{
			edges_cloud.push_back(edge_array1[i].points[0]);
//			edges_cloud.push_back(edge_array1[i].points[1]);
//			edges_cloud.push_back(edge_array1[i].points[1]);
		}
	if(!edge_array2.empty())
		for(int i=0; i<2; i++)
		{
			edges_cloud.push_back(edge_array2[i].points[0]);
//			edges_cloud.push_back(edge_array2[i].points[1]);
//			edges_cloud.push_back(edge_array2[i].points[2]);
		}
	return	edges_cloud;
}

										


//-------------------数学计算部分------------------------------------------------
double CurbDetection::get_dev_x(pcl::PointCloud<pcl::PointXYZI> cloud, int idx, int len)
{
//	if( (idx+len) >= cloud.size() )
//		return -1;
		
	double sum_x;
	double mean_x;
	double dx;
	double sum_dx2;
	
	if(len>0)
	{
		for(int i=idx; i<(idx+len); i++)
			sum_x += cloud.points[i].x;
		
		mean_x = sum_x/len;
	
		for(int i=idx; i<(idx+len); i++)
		{
			dx = cloud.points[i].x - mean_x;
			sum_dx2 += dx*dx;
		}
	}
	else
	{
		for(int i=(idx+len); i<idx; i++)
			sum_x += cloud.points[i].x;
		
		mean_x = sum_x/abs(len);//注意len为负数
	
		for(int i=(idx+len); i<idx; i++)
		{
			dx = cloud.points[i].x - mean_x;
			sum_dx2 += dx*dx;
		}
	}

	return	sum_dx2;
}

double CurbDetection::get_dev_y(pcl::PointCloud<pcl::PointXYZI> cloud, int idx, int len)
{
//	if( (idx+len) >= cloud.size() )
//		return -1;
		
	double sum_y;
	double mean_y;
	double dy;
	double sum_dy2;
	
	if(len>0)
	{
		for(int i=idx; i<(idx+len); i++)
			sum_y += cloud.points[i].y;
		
		mean_y = sum_y/len;
	
		for(int i=idx; i<(idx+len); i++)
		{
			dy = cloud.points[i].y - mean_y;
			sum_dy2 += dy*dy;
		}
	}
	else
	{
		for(int i=(idx+len); i<idx; i++)
			sum_y += cloud.points[i].y;
		
		mean_y = sum_y/abs(len);//注意len为负数
	
		for(int i=(idx+len); i<idx; i++)
		{
			dy = cloud.points[i].y - mean_y;
			sum_dy2 += dy*dy;
		}
	}

	return	sum_dy2;
}

bool CurbDetection::slope_filter(pcl::PointXYZI p, double threshold)
{
	double x = p.x;
	double y = p.y;
	double slope = x/(y+0.001);
	if(abs(slope)>threshold) return true;
	else	return false;	
}

bool CurbDetection::dist_filter(pcl::PointXYZI p, double threshold)
{
	double x = p.x;
	double y = p.y;
	double z = p.z;
	double dist2 = x*x + y*y + z*z;
	double dist = sqrt(dist2);
	if(dist<threshold) return true;
	else	return false;
}

bool CurbDetection::is_points_available(pcl::PointXYZI p)
{
	if(	int(p.x)==INVAILD_LABEL &&
		int(p.y)==INVAILD_LABEL &&
		int(p.z)==INVAILD_LABEL)
		return	false;
	else	return true;
}
//--------------------数学计算部分----------------------------------//



//--------------------可视化部分-----------------------------------
void CurbDetection::show_line(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ring,
				vector<pcl::PointXYZI> line_points)
{
	pcl::PointXYZRGB p;
	for(int point_id=0; point_id<line_points.size(); point_id++)
	{
		p.x = line_points[point_id].x;
		p.y = line_points[point_id].y;
		p.z = line_points[point_id].z;
		p.rgb = *reinterpret_cast<float*>(&rainbow[2]);
		cloud_ring->push_back(p);
	}
}

void CurbDetection::show_edge(	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ring,
				vector<pcl::PointCloud<pcl::PointXYZI> > edge_points)
{
	pcl::PointXYZRGB p;
	for(int ring_id=0; ring_id<ARRAY_SIZE; ring_id++)
	{
		for(int edge_id=0; edge_id<edge_points[ring_id].size(); edge_id++)
		{
			if( !is_points_available(edge_points[ring_id].points[edge_id]) )	
				continue;
			p.x = edge_points[ring_id].points[edge_id].x;
			p.y = edge_points[ring_id].points[edge_id].y;
			p.z = edge_points[ring_id].points[edge_id].z;
			p.rgb = *reinterpret_cast<float*>(&rainbow[3]);
			cloud_ring->push_back(p);
		}
	}
}

void CurbDetection::visualize(	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array, 
								vector<pcl::PointCloud<pcl::PointXYZI> > edge_head_right,
								vector<pcl::PointCloud<pcl::PointXYZI> > edge_head_left,
								vector<pcl::PointCloud<pcl::PointXYZI> > edge_rear_right,
								vector<pcl::PointCloud<pcl::PointXYZI> > edge_rear_left)
{
	pcl::PointXYZRGB p;
	for(int cloud_id=0; cloud_id<ARRAY_SIZE; cloud_id++)
	{
		for(int point_id=0; point_id<cloud_array[cloud_id].size(); point_id++)
		{
			p.x = cloud_array[cloud_id].points[point_id].x;
			p.y = cloud_array[cloud_id].points[point_id].y;
			p.z = cloud_array[cloud_id].points[point_id].z;
			p.rgb = *reinterpret_cast<float*>(&rainbow[0]);
			cloud_ring->push_back(p);
		}
	}
	show_edge(cloud_ring,edge_head_right);
	show_edge(cloud_ring,edge_head_left);
//	show_edge(cloud_ring,edge_rear_right);
//	show_edge(cloud_ring,edge_rear_left);
	
//	show_line(cloud_ring,line_head_right);
//	show_line(cloud_ring,line_head_left);
//	show_line(cloud_ring,line_rear_right);
//	show_line(cloud_ring,line_rear_left);
#if PUB_CLOUD_RING
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_ring, cloud_to_pub);
    pub_cloud_ring.publish(cloud_to_pub);
#endif  
}
//----------------------可视化部分---------------------------------------------------//
