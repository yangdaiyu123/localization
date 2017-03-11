#ifndef _CLOUD_ACCUMULATOR_H_
#define _CLOUD_ACCUMULATOR_H_


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

#include <pcl_ros/point_cloud.h>
#include "point_xyzo.h"

#include <vector>
#include <sstream>

class Cloudaccumulator
{
public:
	Cloudaccumulator();
	void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input);
	void rtkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
	void estPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
	
	void frontCurbCallback(const OPointCloud::ConstPtr &input);
	void rearCurbCallback(const OPointCloud::ConstPtr &input);
	void signCallback(const OPointCloud::ConstPtr &input);

private:
	pcl::PointCloud<PointXYZO> cloud_new;
	pcl::PointCloud<PointXYZO> cloud_in_map;
	int queue_size;
	geometry_msgs::PoseStamped rtk_pose;
	geometry_msgs::PoseStamped est_pose;
	
	ros::NodeHandle nh;
	
	ros::Subscriber sub_rtk_pose;
	ros::Subscriber sub_est_pose;
	bool use_rtk;
	
	ros::Subscriber sub_pulse;
	ros::Subscriber sub_imu;
	
	ros::Subscriber sub_front_curb;
	ros::Subscriber sub_rear_curb;
	ros::Subscriber sub_sign;

	ros::Publisher pub_cloud_sum;


	Eigen::Affine3f b_to_m;
	pcl::PointCloud<PointXYZO>::Ptr cloud_sum;
	float odom_sum;
		


	std::vector<int> fifo_size;
	tf::StampedTransform trans_front;
};







#endif
