#ifndef _CURB_DETECTOR_H
#define	_CURB_DETECTOR_H


#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus//model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>


#include <pcl_ros/point_cloud.h>
#include <point_types.h>
#include <cloud_operation.h>
#include <point_xyzo.h>


#include <string>
#include <iostream>


using namespace std;
using namespace pcl;

//const int	INVAILD_LABEL=	-100;

class CurbDetector
{
public:
	CurbDetector(){};
	~CurbDetector(){};

	void setInputCloud(const VPointCloud::ConstPtr &inMsg);
	void setArraySize(int array_size){ARRAY_SIZE = array_size;}
	void setWindowSize(int window_size){WINDOW_SIZE = window_size;}
	void setEdgeTH(int edge_th){EDGE_TH = edge_th;}
	void setAngleeTH(int angle_th){ANGLE_TH = angle_th;}


private:

	vector<pcl::PointCloud<pcl::PointXYZI> > sortCloud();

	VPointCloud::ConstPtr cloud_in_;
	int ARRAY_SIZE;
	int WINDOW_SIZE;
	int EDGE_TH;
	int ANGLE_TH;

};



#endif
