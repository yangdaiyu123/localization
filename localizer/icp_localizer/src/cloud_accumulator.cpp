#include "cloud_accumulator.h"

using namespace std;

const double ODOMETRY_FACTOR = 0.0210386;

Cloudaccumulator::Cloudaccumulator()
{
	sub_pulse = nh.subscribe("/pulse", 2, &Cloudaccumulator::pulseCallback, this);
	sub_rtk_pose = nh.subscribe("/rtk_pose", 2, &Cloudaccumulator::rtkPoseCallback, this);
	sub_est_pose = nh.subscribe("/icp_pose", 2, &Cloudaccumulator::estPoseCallback, this);

	sub_front_curb = nh.subscribe("front_curb_raw",2, &Cloudaccumulator::frontCurbCallback, this);
	sub_rear_curb = nh.subscribe("rear_curb_raw",2, &Cloudaccumulator::rearCurbCallback, this);
	sub_sign = nh.subscribe("sign_points",2, &Cloudaccumulator::signCallback, this);

	pub_cloud_sum = nh.advertise<sensor_msgs::PointCloud2>("feature_points_sum", 2);

	cloud_sum = pcl::PointCloud<PointXYZO>::Ptr(new pcl::PointCloud<PointXYZO>);
	
	use_rtk = true;
	is_inited = false;
	rec_est_pose = false;
	odom_sum = 0.0;

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
}

void Cloudaccumulator::frontCurbCallback(const OPointCloud::ConstPtr &input)
{
//	if(use_rtk)
//		cout<<"rtk\n"<<b_to_m.matrix()<<endl;	
//	else
//		cout<<"est\n"<<b_to_m.matrix()<<endl;	

	if(input->empty() && is_inited)
	{
		cloud_in_map.header.frame_id = "map";
		pcl_conversions::toPCL(ros::Time::now(), cloud_in_map.header.stamp);
		sensor_msgs::PointCloud2 cloud_to_pub;
		pcl::toROSMsg(cloud_in_map, cloud_to_pub);
		pub_cloud_sum.publish(cloud_to_pub);
		return;
	}

	PointXYZO po;
	cloud_new.clear();
	for(int i=0; i<input->points.size(); i++)
	{
		po.x = input->points[i].x;
		po.y = input->points[i].y;
		po.z = input->points[i].z;
		po.orientation = input->points[i].orientation;
		cloud_new.push_back(po);
	}
	pcl::transformPointCloud(cloud_new, cloud_new, b_to_m);
	cloud_in_map += cloud_new;
	fifo_size.push_back(cloud_new.size());
	
	if (fifo_size.size()>100)
	{
		cloud_in_map.erase(cloud_in_map.begin(), cloud_in_map.begin() + fifo_size[0]);
		fifo_size.erase(fifo_size.begin());
	}
	
	if(is_inited)
	{
		cloud_in_map.header.frame_id = "map";
		pcl_conversions::toPCL(ros::Time::now(), cloud_in_map.header.stamp);
		sensor_msgs::PointCloud2 cloud_to_pub;
		pcl::toROSMsg(cloud_in_map, cloud_to_pub);
		pub_cloud_sum.publish(cloud_to_pub);
	}

	
	
//	for(int i=0; i<input->points.size(); i++)
//		cout<<"i: "<<i<<"x: "<<input->points[i].x<<" "<<"y: "<<input->points[i].y<<" "<<"orientation: "<<input->points[i].orientation<<endl;

}

void Cloudaccumulator::rearCurbCallback(const OPointCloud::ConstPtr &input)
{



}

void Cloudaccumulator::signCallback(const OPointCloud::ConstPtr &input)
{


}

void Cloudaccumulator::rtkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
	double roll, pitch, yaw;
	tf::Quaternion q;
    tf::quaternionMsgToTF(input->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double x = input->pose.position.x;
    double y = input->pose.position.y;
    double z = input->pose.position.z;

    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << x, y, z;
	trans.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

	if( !is_inited || !rec_est_pose )	b_to_m = trans;
	
//	cout<<"rtk\t"<<"x: "<<x<<" "<<"y: "<<y<<" "<<"z: "<<z<<" "<<"yaw: "<<yaw<<endl;
}

void Cloudaccumulator::estPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
	double roll, pitch, yaw;
	tf::Quaternion q;
    tf::quaternionMsgToTF(input->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double x = input->pose.position.x;
    double y = input->pose.position.y;
    double z = input->pose.position.z;

    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << x, y, z;
	trans.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
	
	rec_est_pose = true;
	if(is_inited)	b_to_m = trans;
	
//	cout<<"est\t"<<"x: "<<x<<" "<<"y: "<<y<<" "<<"z: "<<z<<" "<<"yaw: "<<yaw<<endl;
}

void Cloudaccumulator::pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	if( !is_inited )
		odom_sum += odom_inc;
	
	if(odom_sum > 60)
	{
		is_inited = true;
		odom_sum = 0;
	}
		
		
//	cout<<"odom_sum: "<<odom_sum<<endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Cloudaccumulator");

	Cloudaccumulator localizer;

	return 0;
}
