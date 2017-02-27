#include "ros/ros.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <tf/tf.h>

int
main (int argc, char** argv)
{
	ros::init(argc, argv, "ndt_demo2");
	ros::NodeHandle nh;

	// Loading first scan of room.
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wlh/pcd/826_61.pcd", *input_cloud) == -1)
	{
	PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
	return (-1);
	}
	// Filtering input scan to roughly 10% of original size to increase speed of registration.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
	approximate_voxel_filter.setInputCloud (input_cloud);
	approximate_voxel_filter.filter (*filtered_cloud);

	Eigen::Affine3f trans0 = Eigen::Affine3f::Identity();
	trans0.translation() << -41320.719, -3091.3975, 0;
	pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, trans0);

	// Loading second scan of room from new perspective.
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << 1, 5, 0;
	trans.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	pcl::transformPointCloud(*filtered_cloud, *target_cloud, trans);


	// Initializing Normal Distributions Transform (NDT).
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

	// Setting scale dependent NDT parameters
	// Setting minimum transformation difference for termination condition.
	ndt.setTransformationEpsilon (0.01);
	// Setting maximum step size for More-Thuente line search.
	ndt.setStepSize (0.1);
	//Setting Resolution of NDT grid structure (VoxelGridCovariance).
	ndt.setResolution (1.0);

	// Setting max number of registration iterations.
	ndt.setMaximumIterations (35);

	// Setting point cloud to be aligned.
	ndt.setInputSource (filtered_cloud);
	// Setting point cloud to be aligned to.
	ndt.setInputTarget (target_cloud);

	// Set initial alignment estimate found using robot odometry.
	Eigen::AngleAxisf init_rotation (0.02, Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (0.5, 6, 0);
	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

	// Calculating required rigid transform to align the input cloud to the target cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align (*output_cloud, init_guess);

	std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
		    << " score: " << ndt.getFitnessScore () << std::endl;

	Eigen::Matrix4f t = ndt.getFinalTransformation();
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
	std::cout<<"x: "<<x<<"\ty: "<<y<<"\tz: "<<z<<std::endl;
	std::cout<<"roll: "<<roll<<"\tpitch: "<<pitch<<"\tyaw: "<<yaw<<std::endl;

	// Transforming unfiltered, input cloud using found transform.
	pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

	// Saving transformed input cloud.
//	pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

	// Initializing point cloud visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer>
	viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer_final->setBackgroundColor (0, 0, 0);

	// Coloring and visualizing target cloud (red).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (target_cloud, 255, 0, 0);
	viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		                                          1, "target cloud");

	// Coloring and visualizing transformed input cloud (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color (output_cloud, 0, 255, 0);
	viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		                                          1, "output cloud");

	// Coloring and visualizing transformed input cloud (green).
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_color (input_cloud, 0, 0, 255);
	viewer_final->addPointCloud<pcl::PointXYZ> (input_cloud, input_color, "input_cloud");
	viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
		                                          1, "input cloud");

	// Starting visualizer
//	viewer_final->addCoordinateSystem (1.0, "global");
	viewer_final->initCameraParameters ();

	// Wait until visualizer window is closed.
	while (!viewer_final->wasStopped ())
	{
	viewer_final->spinOnce (100);
	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return (0);
}
