#ifndef _ICP_LOCALIZER_H_
#define _ICP_LOCALIZER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>
#include <pcl/visualization/impl/point_cloud_color_handlers.hpp>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <sstream>

#include "cloud_accumulator.h"
#include <pcl_ros/point_cloud.h>
#include "point_xyzo.h"

#include <vector>

const double ODOMETRY_FACTOR = 0.0210386;

class NDTLocalizer
{
public:
	NDTLocalizer();
	void mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void featurePointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
	void rtkCallback(const geometry_msgs::PoseStamped::ConstPtr& input);
	void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input);
	void imuCallback(const sensor_msgs::Imu::ConstPtr& input);



private:
	ros::NodeHandle nh;
	ros::Subscriber sub_map;
	ros::Subscriber sub_points;
	ros::Subscriber sub_pose;
	ros::Subscriber sub_pulse;
	ros::Subscriber sub_imu;
	ros::Subscriber sub_rtk;


	ros::Publisher pub_icp_pose;
	ros::Publisher pub_pose_fixed;

	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	pcl::PointCloud<pcl::PointXYZ> map;
	// Default values for NDT
	int maximum_iterations;
	double ndt_res;
	double transformation_epsilon;
	double step_size;
	double fitness_score;
	
	
	bool map_loaded;
	bool is_inited;
	bool rece_imu;
	
	double curr_turn, last_turn, turn_inc;
	double curr_odom, last_odom, odom_inc;
	double curr_yaw, last_yaw;
	double odom_t, turn_t;
	
	Eigen::Matrix4f fix_matrix;
	geometry_msgs::PoseStamped curr_pose;
	geometry_msgs::PoseStamped last_pose;
	geometry_msgs::PoseStamped pred_pose;
	geometry_msgs::PoseStamped rtk_pose;
};












#endif
