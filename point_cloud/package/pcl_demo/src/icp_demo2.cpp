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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <sstream>

using namespace std;

pcl::visualization::PCLVisualizer p;
double tx,ty,tz;
double rx,ry,rz;
double init_tx,init_ty,init_tz;
double init_rx,init_ry,init_rz;
double TransformationEpsilon;
double MaxCorrespondenceDistance;
double EuclideanFitnessEpsilon;
int MaximumIterations;
string pcd_file;
bool show_src,show_tgt,show_reg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_demo2");
	ros::NodeHandle nh;

	ros::NodeHandle pnh("~");
	pnh.param<std::string>("pcd_file",pcd_file, "/home/wlh/pcd/826_61.pcd");
	pnh.param("tx", tx, 1.0);
	pnh.param("ty", ty, 2.0);
	pnh.param("tz", tz, 0.0);
	pnh.param("rx", rx, 0.0);
	pnh.param("ry", ry, 0.0);
	pnh.param("rz", rz, 0.1);
	pnh.param("init_tx", init_tx, 1.0);
	pnh.param("init_ty", init_ty, 2.0);
	pnh.param("init_tz", init_tz, 0.0);
	pnh.param("init_rx", init_rx, 0.0);
	pnh.param("init_ry", init_ry, 0.0);
	pnh.param("init_rz", init_rz, 0.1);
	pnh.param("TransformationEpsilon", TransformationEpsilon, 1e-10);
	pnh.param("MaxCorrespondenceDistance", MaxCorrespondenceDistance, 100.0);
	pnh.param("EuclideanFitnessEpsilon", EuclideanFitnessEpsilon, 0.1);
	pnh.param("MaximumIterations", MaximumIterations, 500);

	pnh.param("show_src", show_src, false);
	pnh.param("show_tgt", show_tgt, true);
	pnh.param("show_reg", show_reg, true);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_registration (new pcl::PointCloud<pcl::PointXYZ>);

	// load pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud_source)==-1) {
		std::cout << "load source failed!" << std::endl;
		return -1;
	}
	std::cout << "source loaded!" << std::endl;

	Eigen::Affine3f trans0 = Eigen::Affine3f::Identity();
	trans0.translation() << -51.755543, -58.720894, 0;
	pcl::transformPointCloud(*cloud_source, *cloud_source, trans0);

	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << tx, ty, tz;
	trans.rotate(Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_source, *cloud_trans, trans);

	Eigen::Translation3f init_translation (init_tx, init_ty, init_tz);
	Eigen::AngleAxisf init_rotation (init_rz, Eigen::Vector3f::UnitZ ());
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

	double t0 = ros::Time::now().toSec();

	// ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_source);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_trans);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_trans);
	icp.setMaxCorrespondenceDistance(MaxCorrespondenceDistance);
	icp.setTransformationEpsilon(TransformationEpsilon);
	icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);
	icp.setMaximumIterations(MaximumIterations);
	icp.align(*cloud_registration,init_guess);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	double t1 = ros::Time::now().toSec();
	std::cout<<"time cost: "<<t1-t0<<std::endl;

	Eigen::Matrix4f t = icp.getFinalTransformation();
	tf::Matrix3x3 mat_l;  // localizer
	mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
	               static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
	               static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

	double x,y,z;
	double roll,pitch,yaw;
	// Update localizer_pose
	x = t(0, 3);
	y = t(1, 3);
	z = t(2, 3);
	mat_l.getRPY(roll, pitch, yaw, 1);
//		std::cout << transformation << std::endl;
	std::cout<<"x: "<<x<<"\ty: "<<y<<"\tz: "<<z<<std::endl;
	std::cout<<"roll: "<<roll<<"\tpitch: "<<pitch<<"\tyaw: "<<yaw<<std::endl;

	// display
	//pcl::visualization::PCLVisualizer p;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> reg (cloud_registration, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt (cloud_trans, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src (cloud_source, 0, 0, 255);
	if(show_reg)	p.addPointCloud(cloud_registration, reg,"registration");
	if(show_tgt)	p.addPointCloud(cloud_trans, tgt, "target");
	if(show_src)	p.addPointCloud(cloud_source, src, "source");
	p.spin();

	return 0;


}
