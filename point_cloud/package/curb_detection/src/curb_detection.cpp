#include <ros/ros.h>
#include "curb_detection.h"

using namespace std;
using namespace cloud_operation;

pcl::visualization::CloudViewer MyViewer("cloud");

CurbDetection::CurbDetection()
{
	sub_front_cloud = nh.subscribe("front_points_in_base_link", 2, &CurbDetection::frontPointCloudCallback,this);
	sub_rear_cloud = nh.subscribe("rear_points_in_base_link", 2, &CurbDetection::rearPointCloudCallback,this);
	sub_top_cloud = nh.subscribe("top_points_in_base_link", 2, &CurbDetection::topPointCloudCallback,this);
	pub_front_curb = nh.advertise<sensor_msgs::PointCloud2>("front_curb_raw",2);
	pub_rear_curb = nh.advertise<sensor_msgs::PointCloud2>("rear_curb_raw",2);
	pub_sign = nh.advertise<sensor_msgs::PointCloud2>("sign_points",2);

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
	detector->setInputCloud(inMsg);
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array = cloudToArray(inMsg,12);
	vector<int> head_idx,rear_idx;
	getHeadRearIdx(cloud_array, head_idx, rear_idx);
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted = sortCloudArray(cloud_array, head_idx, rear_idx);

	vector<pcl::PointCloud<pcl::PointXYZI> > head_right_edges = searchHeadRight(cloud_sorted,2, 8, 60);
	vector<pcl::PointCloud<pcl::PointXYZI> > head_left_edges = searchHeadLeft(cloud_sorted,2, 8, 60);
	pcl::PointCloud<pcl::PointXYZRGB> color_cloud = arrayToColorCloud(cloud_array, 0);
//	markPoints(color_cloud, head_right_edges, 3);
//	markPoints(color_cloud, head_left_edges, 2);
//	pcl::PointCloud<pcl::PointXYZRGB> head_right_curb = arrayToColorCloud(head_right_edges, 3);
//	pcl::PointCloud<pcl::PointXYZRGB> head_left_curb = arrayToColorCloud(head_left_edges, 3);
//	pcl::PointCloud<pcl::PointXYZRGB> front_curb = head_left_curb + head_right_curb;

//	front_curb.header.frame_id = "base_link";
//	sensor_msgs::PointCloud2 cloud_to_pub;
//  pcl::toROSMsg(front_curb, cloud_to_pub);
//  pub_front_curb.publish(cloud_to_pub);



	pcl::PointCloud<pcl::PointXYZ> head_right_curb = arrayToCloudXYZ(head_right_edges);
	pcl::PointCloud<pcl::PointXYZ> head_left_curb = arrayToCloudXYZ(head_left_edges);

	filter::remove_invalid_points(head_right_curb);
	filter::remove_invalid_points(head_left_curb);
	filter::dist_filter(head_right_curb);
	filter::dist_filter(head_left_curb);

	Eigen::VectorXf model_l,model_r;

    pcl::PointCloud<pcl::PointXYZ> inlier_points1 = filter::ransac_fit_line(head_right_curb);
    pcl::PointCloud<pcl::PointXYZ> inlier_points2 = filter::ransac_fit_line(head_left_curb);
    pcl::PointCloud<pcl::PointXYZ> front_curb = inlier_points1 + inlier_points2;

    pcl::PointCloud<PointXYZO> orientation_curb;
    PointXYZO po;

//    cout << model_l[0] << " " << model_l[1] << " " << model_l[2] << " "
//		 << model_l[3] << " " << model_l[4] << " " << model_l[5] << endl;

//    for(int i=0; i<inlier_points1.points.size(); i++)
//    {
//		po.x = inlier_points1.points[i].x;
//		po.y = inlier_points1.points[i].y;
//		po.z = inlier_points1.points[i].z;
//		po.orientation =
//    	orientation_curb
//    }

    markPoints(color_cloud, inlier_points1, 2);
    markPoints(color_cloud, inlier_points2, 3);

	if(!inlier_points1.empty())	right_curb_dist = abs(inlier_points1.points[0].y+2);
    if(!inlier_points2.empty())	left_curb_dist = abs(inlier_points2.points[0].y+2);

//    MyViewer.showCloud(color_cloud.makeShared(),"cloud");

	if(front_curb.empty())	return;
	front_curb.header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(front_curb, cloud_to_pub);
    pub_front_curb.publish(cloud_to_pub);
}

void CurbDetection::rearPointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array = cloudToArray(inMsg,12);
	vector<int> head_idx,rear_idx;
	getHeadRearIdx(cloud_array, head_idx, rear_idx);
	vector<pcl::PointCloud<pcl::PointXYZI> > cloud_sorted = sortCloudArrayReverse(cloud_array, head_idx, rear_idx);
	vector<pcl::PointCloud<pcl::PointXYZI> > rear_right_edges = searchRearRight(cloud_sorted,2, 8, 60);
	vector<pcl::PointCloud<pcl::PointXYZI> > rear_left_edges = searchRearLeft(cloud_sorted,2, 8, 60);
	pcl::PointCloud<pcl::PointXYZRGB> color_cloud = arrayToColorCloud(cloud_array, 0);
//	markPoints(color_cloud, rear_right_edges, 3);
//	markPoints(color_cloud, rear_left_edges, 2);
//	pcl::PointCloud<pcl::PointXYZRGB> rear_right_curb = arrayToColorCloud(rear_right_edges, 3);
//	pcl::PointCloud<pcl::PointXYZRGB> rear_left_curb = arrayToColorCloud(rear_left_edges, 3);
//	pcl::PointCloud<pcl::PointXYZRGB> rear_curb = rear_left_curb + rear_right_curb;



	pcl::PointCloud<pcl::PointXYZ> rear_right_curb = arrayToCloudXYZ(rear_right_edges);
	pcl::PointCloud<pcl::PointXYZ> rear_left_curb = arrayToCloudXYZ(rear_left_edges);
	filter::remove_invalid_points(rear_right_curb);
	filter::remove_invalid_points(rear_left_curb);
	filter::dist_filter(rear_right_curb);
	filter::dist_filter(rear_left_curb);
    pcl::PointCloud<pcl::PointXYZ> inlier_points1 = filter::ransac_fit_line(rear_right_curb);
    pcl::PointCloud<pcl::PointXYZ> inlier_points2 = filter::ransac_fit_line(rear_left_curb);
    pcl::PointCloud<pcl::PointXYZ> rear_curb = inlier_points1 + inlier_points2;
//    markPoints(color_cloud, inlier_points1, 5);
//    markPoints(color_cloud, inlier_points2, 4);
//    MyViewer.showCloud(color_cloud.makeShared(),"cloud");

	if(rear_curb.empty()) return;
	rear_curb.header.frame_id = "base_link";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(rear_curb, cloud_to_pub);
	pub_rear_curb.publish(cloud_to_pub);
}

void CurbDetection::topPointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
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

	pcl::PointCloud<pcl::PointXYZRGB> color_cloud = arrayToColorCloud(cloud_array, 0);

	//创建一个模型参数对象，用于记录结果
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//inliers表示误差能容忍的点 记录的是点云的序号
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// 创建一个分割器
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory-设置目标几何形状
	seg.setModelType (pcl::SACMODEL_LINE);
	//分割方法：随机采样法
	seg.setMethodType (pcl::SAC_RANSAC);
	//设置误差容忍范围
	seg.setDistanceThreshold (0.03);
	seg.setMaxIterations(80);


	pcl::PointCloud<pcl::PointXYZ> sign_points;
	for(int i=3; i<8; i++)
	{
		//输入点云
		if(cloud_array[i].points.size()<50)	continue;
		seg.setInputCloud (cloud_array[i].makeShared());
		//分割点云
		seg.segment (*inliers, *coefficients);

		if(abs(coefficients->values[4])>0.9 && abs(coefficients->values[5])<0.1 && inliers->indices.size()>=20)
		{
			cout<< inliers->indices.size ()<<" "<<coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " "
			 	<< coefficients->values[3] << " " << coefficients->values[4] << " " << coefficients->values[5] << endl;
			pcl::ExtractIndices<pcl::PointXYZI> extract;
			extract.setInputCloud(cloud_array[i].makeShared());
			extract.setIndices(inliers);
			pcl::PointCloud<pcl::PointXYZI> cloud_line;
			extract.filter(cloud_line);
			markPoints(color_cloud, cloud_line, 3);

			for(int idx=0; idx<cloud_line.points.size(); idx++)
			{
				pcl::PointXYZ p;
				p.x = cloud_line.points[idx].x; p.y = cloud_line.points[idx].y; p.z = cloud_line.points[idx].z;
				sign_points.push_back(p);
			}
		}
	}

	if(!sign_points.empty())
	{
//		markPoints(color_cloud, sign_points, 3);

		sign_points.header.frame_id = "base_link";
		sensor_msgs::PointCloud2 cloud_to_pub;
		pcl::toROSMsg(sign_points, cloud_to_pub);
		pub_sign.publish(cloud_to_pub);
	}
	MyViewer.showCloud(color_cloud.makeShared(),"cloud");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "curb_detection");

	CurbDetection obj;

	return 0;
}
