#include <ros/ros.h>
#include "curb_detection.h"

using namespace std;
using namespace cloud_operation;

#define DEBUG	0

#if DEBUG
pcl::visualization::PCLVisualizer p;
#endif

CurbDetection::CurbDetection()
{
	sub_front_cloud = nh.subscribe("front_points_in_base_link", 2, &CurbDetection::frontPointCloudCallback,this);
	sub_rear_cloud = nh.subscribe("rear_points_in_base_link", 2, &CurbDetection::rearPointCloudCallback,this);
	sub_top_cloud = nh.subscribe("top_points_in_base_link", 2, &CurbDetection::topPointCloudCallback,this);
	pub_front_curb = nh.advertise<sensor_msgs::PointCloud2>("front_curb_raw",2);
	pub_rear_curb = nh.advertise<sensor_msgs::PointCloud2>("rear_curb_raw",2);
	pub_sign = nh.advertise<sensor_msgs::PointCloud2>("sign_points",2);
	pub_marker = nh.advertise<sensor_msgs::PointCloud2>("marker_points",2);

	detector = new CurbDetector;

	left_curb_dist = 10;
	right_curb_dist =10;

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
}

CurbDetection::~CurbDetection()
{
	delete detector;
}


void CurbDetection::frontPointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
//顺序重排
//------------------------------------------------------------------------------------------------------------
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array = cloudToArray(inMsg,12);
	vector<int> head_idx,rear_idx;
	getHeadRearIdx(cloud_array, head_idx, rear_idx);
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted = sortCloudArray(cloud_array, head_idx, rear_idx);

//搜索跳变点
//------------------------------------------------------------------------------------------------------------
	vector<pcl::PointCloud<pcl::PointXYZI> > head_right_edges = searchHeadRight(cloud_sorted,2, 8, 60);
	vector<pcl::PointCloud<pcl::PointXYZI> > head_left_edges = searchHeadLeft(cloud_sorted,2, 8, 60);
	pcl::PointCloud<pcl::PointXYZ> head_right_curb = arrayToCloudXYZ(head_right_edges);
	pcl::PointCloud<pcl::PointXYZ> head_left_curb = arrayToCloudXYZ(head_left_edges);

//去除干扰点
//------------------------------------------------------------------------------------------------------------
	filter::remove_invalid_points(head_right_curb);
	filter::remove_invalid_points(head_left_curb);
	filter::dist_filter(head_right_curb);
	filter::dist_filter(head_left_curb);
	Eigen::VectorXf model_l,model_r;
    pcl::PointCloud<pcl::PointXYZ> inlier_points1 = filter::ransac_fit_line(head_right_curb,model_r);
    pcl::PointCloud<pcl::PointXYZ> inlier_points2 = filter::ransac_fit_line(head_left_curb,model_l);

//生成向量点云
//------------------------------------------------------------------------------------------------------------
//	model_coefficients中前3个元素表示单位向量的起点，后3个元素代表单位向量在3个方向的分量
	front_curb.clear();
	markOrientation(front_curb, inlier_points1, model_r);
	markOrientation(front_curb, inlier_points2, model_l);

//道路边界
//------------------------------------------------------------------------------------------------------------
	if(!inlier_points1.empty())	right_curb_dist = abs(inlier_points1.points[0].y+2);
    if(!inlier_points2.empty())	left_curb_dist = abs(inlier_points2.points[0].y+2);
    
//路面标志提取
	marker_points = searchHeadMarkerPoints(cloud_sorted, 28, -2, 4);

//发布路沿点云
//------------------------------------------------------------------------------------------------------------
	front_curb.header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(front_curb, cloud_to_pub);
	pub_front_curb.publish(cloud_to_pub);
	
	marker_points.header.frame_id = "base_link";
	pcl::toROSMsg(marker_points, cloud_to_pub);
	pub_marker.publish(cloud_to_pub);

//显示
//------------------------------------------------------------------------------------------------------------
//    pcl::PointCloud<pcl::PointXYZRGB> color_cloud = arrayToColorCloud(cloud_array, 0);
//    markPoints(color_cloud, inlier_points1, 2);
//    markPoints(color_cloud, inlier_points2, 3);
    //    MyViewer.showCloud(color_cloud.makeShared(),"cloud");
#if DEBUG
	pcl::visualization::PointCloudColorHandlerCustom<PointXYZO> fr (front_curb.makeShared(), 255, 0, 0);
	p.removePointCloud("front");
	p.addPointCloud(front_curb.makeShared(), fr, "front");

	pcl::visualization::PointCloudColorHandlerCustom<PointXYZO> re (rear_curb.makeShared(), 0, 255, 0);
	p.removePointCloud("rear");
	p.addPointCloud(rear_curb.makeShared(), re, "rear");

	pcl::visualization::PointCloudColorHandlerCustom<PointXYZO> sg (sign_points.makeShared(), 0, 0, 255);
	p.removePointCloud("sign");
	p.addPointCloud(sign_points.makeShared(), sg, "sign");

	p.spinOnce(5);
#endif
}

void CurbDetection::rearPointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
//顺序重排
//------------------------------------------------------------------------------------------------------------
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array = cloudToArray(inMsg,12);
	vector<int> head_idx,rear_idx;
	getHeadRearIdx(cloud_array, head_idx, rear_idx);
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted = sortCloudArrayReverse(cloud_array, head_idx, rear_idx);

//搜索跳变点
//------------------------------------------------------------------------------------------------------------
	vector<pcl::PointCloud<pcl::PointXYZI> > rear_right_edges = searchRearRight(cloud_sorted,2, 8, 60);
	vector<pcl::PointCloud<pcl::PointXYZI> > rear_left_edges = searchRearLeft(cloud_sorted,2, 8, 60);
	pcl::PointCloud<pcl::PointXYZ> rear_right_curb = arrayToCloudXYZ(rear_right_edges);
	pcl::PointCloud<pcl::PointXYZ> rear_left_curb = arrayToCloudXYZ(rear_left_edges);

//去除干扰点
//------------------------------------------------------------------------------------------------------------
	filter::remove_invalid_points(rear_right_curb);
	filter::remove_invalid_points(rear_left_curb);
	filter::dist_filter(rear_right_curb);
	filter::dist_filter(rear_left_curb);
	Eigen::VectorXf model_l,model_r;
    pcl::PointCloud<pcl::PointXYZ> inlier_points1 = filter::ransac_fit_line(rear_right_curb, model_r);
    pcl::PointCloud<pcl::PointXYZ> inlier_points2 = filter::ransac_fit_line(rear_left_curb, model_l);

//生成向量点云
//------------------------------------------------------------------------------------------------------------
	rear_curb.clear();
	markOrientation(rear_curb, inlier_points1, model_r);
	markOrientation(rear_curb, inlier_points2, model_l);

//发布路沿点云
//------------------------------------------------------------------------------------------------------------
	rear_curb.header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(rear_curb, cloud_to_pub);
	pub_rear_curb.publish(cloud_to_pub);

//显示
//------------------------------------------------------------------------------------------------------------
//	pcl::PointCloud<pcl::PointXYZRGB> color_cloud = arrayToColorCloud(cloud_array, 0);
//	markPoints(color_cloud, inlier_points1, 5);
//	markPoints(color_cloud, inlier_points2, 4);
//	MyViewer.showCloud(color_cloud.makeShared(),"cloud");

#if DEBUG
//	pcl::visualization::PointCloudColorHandlerCustom<PointXYZO> re (rear_curb.makeShared(), 0, 255, 0);
//	p.removePointCloud("rear");
//	p.addPointCloud(rear_curb.makeShared(), re, "rear");
//	p.spinOnce(5);
#endif
}

void CurbDetection::topPointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
//顺序重排
//-----------------------------------------------------------------------------------------------------------
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array;
	cloud_array.resize(16);
	for (int i = 1; i < inMsg->points.size(); i++)
	{
		if(inMsg->points[i].ring > 2 && inMsg->points[i].ring < 8)
		{
			int ring_idx = inMsg->points[i].ring;
			pcl::PointXYZI	pt;
			if(inMsg->points[i].y<-10 || inMsg->points[i].y>10) continue;
			pt.x = inMsg->points[i].x;
			pt.y = inMsg->points[i].y;
			pt.z = inMsg->points[i].z;
			pt.intensity = inMsg->points[i].intensity;

			cloud_array[ring_idx].push_back(pt);
		}
	}

//直线检测
//-----------------------------------------------------------------------------------------------------------
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);//创建一个模型参数对象，用于记录结果
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::SACSegmentation<pcl::PointXYZI> seg;// 创建一个分割器
	seg.setOptimizeCoefficients (true);// Optional
	seg.setModelType (pcl::SACMODEL_LINE);// Mandatory-设置目标几何形状
	seg.setMethodType (pcl::SAC_RANSAC);//分割方法：随机采样法
	seg.setDistanceThreshold (0.03);//设置误差容忍范围
	seg.setMaxIterations(80);

//直线筛选
//-----------------------------------------------------------------------------------------------------------
	sign_points.clear();
	for(int i=3; i<8; i++)
	{
		//输入点云
		if(cloud_array[i].points.size()<50)	continue;
		seg.setInputCloud (cloud_array[i].makeShared());
		//分割点云
		seg.segment (*inliers, *coefficients);
		Eigen::VectorXf model = modelConverter( *coefficients );

		if(abs(coefficients->values[4])>0.9 && abs(coefficients->values[5])<0.1 && inliers->indices.size()>=20)
		{
			pcl::ExtractIndices<pcl::PointXYZI> extract;
			extract.setInputCloud(cloud_array[i].makeShared());
			extract.setIndices(inliers);
			pcl::PointCloud<pcl::PointXYZI> cloud_line;
			extract.filter(cloud_line);

			markOrientation(sign_points, cloud_line, model);
		}
	}

//发布点云
//-----------------------------------------------------------------------------------------------------------
	sign_points.header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(sign_points, cloud_to_pub);
	pub_sign.publish(cloud_to_pub);

//显示
//-----------------------------------------------------------------------------------------------------------
//	pcl::PointCloud<pcl::PointXYZRGB> color_cloud = arrayToColorCloud(cloud_array, 0);
//	markPoints(color_cloud, sign_points, 3);
//	MyViewer.showCloud(color_cloud.makeShared(),"cloud");
#if DEBUG
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sign (sign_points.makeShared(), 0, 0, 255);
//	p.removePointCloud("sign");
//	p.addPointCloud(sign_points.makeShared(), sign, "sign");
//	p.spinOnce(5);
#endif
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curb_detection");

	CurbDetection obj;

	return 0;
}
