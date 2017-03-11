#include "icp_localizer.h"

using namespace std;

const double ODOMETRY_FACTOR = 0.0210386;

ICPLocalizer::ICPLocalizer()
{
	sub_map = nh.subscribe("point_map",2, &ICPLocalizer::mapCallback, this);
	sub_points = nh.subscribe("feature_points_sum",2, &ICPLocalizer::featurePointsCallback, this);
	sub_pose = nh.subscribe("estimate_pose", 2, &ICPLocalizer::poseCallback, this);

	pub_icp_pose = nh.advertise<geometry_msgs::PoseStamped>("icp_pose", 2);
//	pub_pose_fixed = nh.advertise<geometry_msgs::PoseStamped>("fix")
	
	maximum_iterations = 500;
	transformation_epsilon = 0.01;
	max_correspondence_distance = 10.0;
	euclidean_fitness_epsilon = 0.1;
	ransac_outlier_rejection_threshold = 1.0;
	fitness_score = 0;

	map_loaded = false;

	ros::MultiThreadedSpinner spinner(8);
	spinner.spin();
}

void ICPLocalizer::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ> map;
    pcl::fromROSMsg(*input, map);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    icp.setInputTarget(map_ptr);
//    std::cout << "setInputTarget finished." << std::endl;

    map_loaded = true;
 
}

void ICPLocalizer::featurePointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ> feature_points;
    pcl::fromROSMsg(*input, feature_points);
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points_ptr(new pcl::PointCloud<pcl::PointXYZ>(feature_points));
    
    icp.setInputSource(feature_points_ptr);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    icp.setMaximumIterations(maximum_iterations);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    icp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);

	icp.align (*output_cloud);  
	
	Eigen::Matrix4f t = icp.getFinalTransformation();
	fix_matrix = t;
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
//	std::cout<<"x: "<<x<<"\ty: "<<y<<"\tz: "<<z<<std::endl;
//	std::cout<<"roll: "<<roll<<"\tpitch: "<<pitch<<"\tyaw: "<<yaw<<std::endl;

}

void ICPLocalizer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
//	cout<<fix_matrix<<endl;	

	geometry_msgs::PoseStamped pose_fixed;
	pose_fixed.pose.position.x = input->pose.position.x + fix_matrix(0, 3);
	pose_fixed.pose.position.y = input->pose.position.y + fix_matrix(1, 3);
	pose_fixed.pose.position.z = input->pose.position.z + fix_matrix(2, 3);
	pose_fixed.pose.orientation = input->pose.orientation;
	
	pose_fixed.header.frame_id = "icp_pose";
	
	static tf::TransformBroadcaster br_filtered;
    tf::Transform transform_filtered;  
    tf::Quaternion q_filtered(pose_fixed.pose.orientation.x, pose_fixed.pose.orientation.y, pose_fixed.pose.orientation.z, pose_fixed.pose.orientation.w);

    transform_filtered.setOrigin(tf::Vector3(pose_fixed.pose.position.x, pose_fixed.pose.position.y,pose_fixed.pose.position.z));

    transform_filtered.setRotation(q_filtered);
    br_filtered.sendTransform( tf::StampedTransform(transform_filtered, input->header.stamp, "map" , "icp_pose"));
	
	pub_icp_pose.publish(pose_fixed);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_localizer");

	ICPLocalizer localizer;

	return 0;
}
