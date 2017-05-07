#ifndef _LCP_MATCHING_H_
#define _LCP_MATCHING_H_

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

#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>

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

template <typename PointT>
class LcpMatching
{
public:
	void set_source_cloud( pcl::PointCloud<PointT> input_src );
	
	void set_target_cloud( pcl::PointCloud<PointT> input_tgt );
	
	void allign_lateral();
	
	pcl::PointCloud<PointT> cloud_translation_step( pcl::PointCloud<PointT> src, double trans_dir, double trans_step );

private:
	pcl::PointCloud<PointT> cloud_src;
	pcl::PointCloud<PointT> cloud_tgt;
	
	void step_to_xy( double trans_dir, double trans_step, double& trans_x, double& trans_y )
	{
		trans_x = trans_step * cos( trans_dir );
		trans_y = trans_step * sin( trans_dir );
	}

};

template <typename PointT>
void LcpMatching<PointT>::set_source_cloud( pcl::PointCloud<PointT> input_src )
{
	cloud_src = input_src;
}

template <typename PointT>	
void LcpMatching<PointT>::set_target_cloud( pcl::PointCloud<PointT> input_tgt )
{
	cloud_tgt = input_tgt;
}

template <typename PointT>
pcl::PointCloud<PointT> LcpMatching<PointT>::cloud_translation_step( pcl::PointCloud<PointT> src, double trans_dir, double trans_step )
{
	double t_x = 0.0;
	double t_y = 0.0;
	pcl::PointCloud<PointT> dst;
	
	step_to_xy( trans_dir, trans_step, t_x, t_y );
	
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << t_x, t_y, 0.0;
	pcl::transformPointCloud(src, dst, trans);
	
	return dst;
}

//template <typename PointT>


#endif
