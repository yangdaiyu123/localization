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

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <sstream>

class ICPLocalizer
{
public:
	ICPLocalizer();
	void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input);
	void gpsCallback(const sensor_msgs::NavSatFixConstPtr& input);
	void yawCallback(const std_msgs::Float64::ConstPtr& input);



private:
	ros::NodeHandle nh;
	ros::Subscriber sub_pulse;
	ros::Subscriber sub_gps;
	ros::Subscriber sub_yaw;
	ros::Subscriber sub_points;
//	ros::Publisher pub_;

};












#endif
