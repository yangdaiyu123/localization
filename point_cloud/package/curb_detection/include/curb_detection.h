#ifndef _CURB_DETECTION_H
#define	_CURB_DETECTION_H

#include "curb_detector.h"
#include <geometry_msgs/PoseStamped.h>

class CurbDetection
{
public:
	CurbDetection();
	~CurbDetection();
	//接收点云的回调函数，所有处理从这里开始
	void frontPointCloudCallback(const VPointCloud::ConstPtr &inMsg);
	void rearPointCloudCallback(const VPointCloud::ConstPtr &inMsg);
	void topPointCloudCallback(const VPointCloud::ConstPtr &inMsg);
	void gpsPoseCallback(geometry_msgs::PoseStamped pose_in);
	pcl::PointCloud<pcl::PointXYZ> ransac_fit_line(pcl::PointCloud<pcl::PointXYZRGB> edges);
	bool is_point_available(pcl::PointXYZRGB p);
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_front_cloud;
	ros::Subscriber sub_rear_cloud;
	ros::Subscriber sub_top_cloud;
	ros::Subscriber sub_gps_pose;
	ros::Publisher pub_front_curb;
	ros::Publisher pub_rear_curb;
	ros::Publisher pub_sign;
	CurbDetector* detector;
	int left_curb_dist;
	int right_curb_dist;
};


#endif
