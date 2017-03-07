#include "icp_localizer.h"

using namespace std;

const double ODOMETRY_FACTOR = 0.0210386;

ICPLocalizer::ICPLocalizer()
{
	sub_pulse = nh.subscribe("/pulse", 2, &ICPLocalizer::pulseCallback, this);
	sub_gps = nh.subscribe("/gps_filtered", 2, &ICPLocalizer::gpsCallback, this);
	sub_heading = nh.subscribe("/yaw_filtered", 2, &ICPLocalizer::headingCallback, this);
	sub_pose = nh.subscribe("/estimate_pose", 2, &ICPLocalizer::poseCallback, this);
	sub_imu = nh.subscribe("/imu_torso/xsens/data", 2, &ICPLocalizer::imuCallback, this);

	sub_front_curb = nh.subscribe("front_curb_raw",2, &ICPLocalizer::frontCurbCallback, this);
	sub_rear_curb = nh.subscribe("rear_curb_raw",2, &ICPLocalizer::rearCurbCallback, this);
	sub_sign = nh.subscribe("sign_points",2, &ICPLocalizer::signCallback, this);

	pub_cloud_sum = nh.advertise<sensor_msgs::PointCloud2>("feature_points_sum", 2);

	front_odom_inc = 0;
	rear_odom_inc = 0;
	sign_odom_inc = 0;
	front_yaw_inc = 0;
	rear_odom_inc = 0;
	sign_yaw_inc = 0;
	cloud_sum = pcl::PointCloud<PointXYZO>::Ptr(new pcl::PointCloud<PointXYZO>);

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
}

void ICPLocalizer::frontCurbCallback(const OPointCloud::ConstPtr &input)
{
	cout<<"front_odom_inc: "<<front_odom_inc<<endl;
	try
	{
		static tf::TransformListener trf_listener1;
		trf_listener1.lookupTransform("map", "base_link", ros::Time(0), trans_front);
		pcl::PointCloud<PointXYZO>::Ptr cloud_out(new pcl::PointCloud<PointXYZO>);

		for (int i=0; i<input->size(); i++)
		{
		    tf::Vector3 pt(input->points[i].x, input->points[i].y, input->points[i].z);
		    tf::Vector3 converted = trans_front * pt;

			double yaw, pitch, roll;
			trans_front.getBasis().getRPY(roll, pitch, yaw);

		    PointXYZO point_in_map;
		    point_in_map.x = converted.x();
		    point_in_map.y = converted.y();
		    point_in_map.z = converted.z();
		    point_in_map.orientation = input->points[i].orientation;

		    cloud_out->push_back(point_in_map);
		}

		cloud_out->header.frame_id = "map";

		*cloud_sum += *cloud_out;
		fifo_size.push_back(cloud_out->size());

		cloud_sum->header.frame_id = "map";
		sensor_msgs::PointCloud2 cloud_to_pub;
		pcl::toROSMsg(*cloud_sum, cloud_to_pub);
		pub_cloud_sum.publish(cloud_to_pub);
	}
	catch (tf::TransformException ex) {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }

	if(fifo_size.size()>100)
	{
		cloud_sum->erase(cloud_sum->begin(), cloud_sum->begin() + fifo_size[0]);
		fifo_size.erase(fifo_size.begin());
	}

	front_odom_inc = 0;
	front_yaw_inc = 0;
}

void ICPLocalizer::rearCurbCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
//	cout<<"rear_odom_inc: "<<rear_odom_inc<<endl;


	rear_odom_inc = 0;
	front_yaw_inc = 0;
}

void ICPLocalizer::signCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
//	cout<<"sign_odom_inc: "<<sign_odom_inc<<endl;


	sign_odom_inc = 0;
	sign_yaw_inc = 0;
}

void ICPLocalizer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
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

	b_to_m = trans;
}

void ICPLocalizer::pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
//	cout<<"pulse0: "<<(int)input->data[0]<<"\tpulse1: "<<(int)input->data[1]<<endl;
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	front_odom_inc += odom_inc;
	rear_odom_inc += odom_inc;
	sign_odom_inc += odom_inc;
}

void ICPLocalizer::gpsCallback(const sensor_msgs::NavSatFixConstPtr& input)
{
//	cout<<"longitude: "<<input->longitude<<"\tlatitude: "<<input->latitude<<endl;
}

void ICPLocalizer::headingCallback(const std_msgs::Float64::ConstPtr& input)
{

}

void ICPLocalizer::imuCallback(const sensor_msgs::Imu::ConstPtr& input)
{
//	cout<<"yaw: "<<input->angular_velocity.z<<endl;
	front_yaw_inc += input->angular_velocity.z;
	rear_yaw_inc += input->angular_velocity.z;
	sign_yaw_inc += input->angular_velocity.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_localizer");

	ICPLocalizer localizer;

	return 0;
}
