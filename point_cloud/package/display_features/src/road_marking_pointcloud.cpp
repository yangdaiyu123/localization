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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Imu.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



using namespace std;
using namespace cv;

const double ODOMETRY_FACTOR = 0.0210386;
double curr_turn = 0.0, last_turn = 0.0, turn_inc = 0.0;
double curr_odom = 0.0, last_odom = 0.0, odom_inc = 0.0;
double curr_yaw = 0.0, last_yaw = 0.0;
double odom_t = 0.0, turn_t = 0.0;
bool rece_imu = false;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_new =  pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sum =  pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher pub_cloud_sum;
int cnt=0;

void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	curr_odom += odom_inc;
	cout<<"curr_odom: "<<curr_odom<<endl;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& input)
{
	if(!rece_imu)
	{
		turn_t = (double) ros::Time::now().toSec();
		rece_imu = true;
		return;
	}
	double dt = (double) ros::Time::now().toSec() - turn_t;
	double yaw_inc = double(input->angular_velocity.z) * dt;
	curr_turn += yaw_inc;
	
	turn_t = (double) ros::Time::now().toSec();
	
//	cout<<"dt: "<<dt<<endl;
//	cout<<"yaw_vel: "<<input->angular_velocity.z<<endl;
//	cout<<"yaw_inc: "<<yaw_inc<<endl;
//	cout<<"curr_turn: "<<curr_turn<<endl;
}

void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::fromROSMsg(*input, *cloud_new);
    
    if(cnt++%4!=0)	return;
    
    //计算位移增量与角度增量
	odom_inc = curr_odom - last_odom;
	turn_inc = curr_turn - last_turn;
	last_odom = curr_odom;
	last_turn = curr_turn;
	
	double inc_x = odom_inc * sin( curr_yaw );
	double inc_y = odom_inc * cos( curr_yaw );
	
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << -inc_x, -inc_y, 0.0;
	transform.rotate(Eigen::AngleAxisf(-turn_inc, Eigen::Vector3f::UnitZ()));

	pcl::transformPointCloud(*cloud_sum, *cloud_sum, transform);
	*cloud_sum = *cloud_sum + *cloud_new;
	
	cloud_sum->header.frame_id = "velodyne";
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(*cloud_sum, cloud_to_pub);
	pub_cloud_sum.publish(cloud_to_pub);
	
	if(curr_odom>10)
		pcl::io::savePCDFileASCII ("cloud_sum.pcd", *cloud_sum);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "road_marking_pointcloud");
	ros::NodeHandle nh;

	ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, &pulseCallback);
	ros::Subscriber sub_imu = nh.subscribe("/imu_torso/xsens/data",2,&imuCallback);
	ros::Subscriber sub_points = nh.subscribe("front_transformed_points",2, &pointCallback);
	pub_cloud_sum = nh.advertise<sensor_msgs::PointCloud2>("accumulated_cloud", 2);
	
	ros::spin();
    
	return 0;
}
