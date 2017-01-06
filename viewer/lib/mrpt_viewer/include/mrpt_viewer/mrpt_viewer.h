#ifndef _MRPT_VIEWER_H_
#define	_MRPT_VIEWER_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include<mrpt/maps/CColouredPointsMap.h>
#include <mrpt/math/utils.h>
#include <mrpt/maps/COctoMap.h>
#include"mrpt/gui.h"
#include"mrpt/opengl.h"
#include"mrpt/opengl/CPlanarLaserScan.h"
#include"mrpt/opengl/CCylinder.h"
#include "mrpt/system/os.h"
#include <mrpt/system/threads.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::opengl;

class MrptViewer
{
public:
	MrptViewer();
	~MrptViewer();

	void display(mrpt::maps::CColouredPointsMap cloud_in);

private:
	mrpt::gui::CDisplayWindow3D win3D;


};



#endif
