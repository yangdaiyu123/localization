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

//pcl::visualization::PCLVisualizer p;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_demo");
	ros::NodeHandle nh;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registration (new pcl::PointCloud<pcl::PointXYZ>);


	// check arguments
	if(argc != 2) {
		std::cout << "ERROR: the number of arguments is illegal!" << std::endl;
		return -1;
	}

	// load pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source)==-1) {
		std::cout << "load source failed!" << std::endl;
		return -1;
	}
	std::cout << "source loaded!" << std::endl;

	Eigen::Affine3f trans0 = Eigen::Affine3f::Identity();
	trans0.translation() << -41320.719, -3091.3975, 0;
	pcl::transformPointCloud(*cloud_source, *cloud_source, trans0);

	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << 0.0, 0, 0;
	trans.rotate(Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*cloud_source, *cloud_trans, trans);


	for(int i=1; i<20; i++)
	{
		Eigen::Affine3f trans = Eigen::Affine3f::Identity();
		trans.translation() << 0.5*i, 1*i, 0;
		trans.rotate(Eigen::AngleAxisf(0.01*i, Eigen::Vector3f::UnitZ()));
		pcl::transformPointCloud(*cloud_source, *cloud_trans, trans);

		Eigen::Translation3f init_translation (0.45*i, 0.8*i, 0);
		Eigen::AngleAxisf init_rotation (0.008*i, Eigen::Vector3f::UnitZ ());
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
		icp.setMaxCorrespondenceDistance(1500);
		icp.setTransformationEpsilon(1e-10);
		icp.setEuclideanFitnessEpsilon(0.1);
		icp.setMaximumIterations(500);
		icp.align(*cloud_source_registration,init_guess);
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

		std::stringstream s0;s0<<i-1;
		std::stringstream s1;s1<<i;


		// display
		//pcl::visualization::PCLVisualizer p;
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_r_h(cloud_source_registration, 255, 0, 0);
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_trans, 0, 255, 0);
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 0, 0, 255);
//		p.addPointCloud(cloud_source_registration, src_r_h,"source_r"+s1.str());
//		p.addPointCloud(cloud_trans, tgt_h, "target"+s1.str());
//		p.addPointCloud(cloud_source, src_h, "source"+s1.str());
//		p.removePointCloud ("source_r"+s0.str());
//		p.removePointCloud ("target"+s0.str());
//		p.removePointCloud ("source"+s0.str());
//		p.spinOnce(100);
	}


	return 0;


}
