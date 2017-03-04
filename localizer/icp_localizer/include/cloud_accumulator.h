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

template <typename PointT>
class Cloudaccumulator
{
public:
	Cloudaccumulator();

private:
	pcl::PointCloud<PointT> cloud_new;
	pcl::PointCloud<PointT> cloud_in_map;
	int queue_size;
	geometry_msgs::PoseStamped pose;
};

template <typename PointT>
Cloudaccumulator<PointT>::Cloudaccumulator()
{
//	queue_size
}






#endif
