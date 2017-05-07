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

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include <tools/lidar_calibrationConfig.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Vector3Stamped.h>

#include <iomanip>

using namespace std;
using namespace message_filters;

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear(new pcl::PointCloud<pcl::PointXYZI>);
ros::Publisher	pub_front;
ros::Publisher	pub_rear;

double front_pitch = 0;//degree
double rear_roll = 0;//degree
double rear_pitch = 0;//degree
double rear_yaw = 0;//degree
double front_tx = 0;
double front_ty = 0;
double front_tz = 0;
double rear_tx = 0;
double rear_ty = 0;
double rear_tz = 0;

const double ODOMETRY_FACTOR = 0.0211;
float curr_odom = 0.0;
float curr_turn = 0.0;
double turn_t;
bool rece_imu = false;

struct OdomStamped
{
	float odom;
	double time_stamp;
};

struct TurnStamped
{
	float turn;
	double time_stamp;
};

vector<OdomStamped> odom_array;
vector<TurnStamped> turn_array;

void synCurbCallback(const sensor_msgs::PointCloud2::ConstPtr& front_cloud_in,
					 const sensor_msgs::PointCloud2::ConstPtr& rear_cloud_in)
{
	pcl::fromROSMsg(*front_cloud_in, *cloud_front);
	pcl::fromROSMsg(*rear_cloud_in, *cloud_rear);
	
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_front->begin(); it != cloud_front->end(); /**/)
	{
		if ( ((*it).x*(*it).x)+((*it).y*(*it).y)  < 1.5)
			it = cloud_front->erase(it);
		else
			++it;
	}
	
	for (pcl::PointCloud<pcl::PointXYZI>::iterator it = cloud_rear->begin(); it != cloud_rear->end(); /**/)
	{
		if ( ((*it).x*(*it).x)+((*it).y*(*it).y)  < 1.5)
			it = cloud_rear->erase(it);
		else
			++it;
	}

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << front_tx, front_ty, front_tz;
	transform.rotate(Eigen::AngleAxisf(front_pitch*M_PI/180, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*cloud_front, *cloud_front, transform);


	Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
	transform2.translation() << rear_tx, rear_ty, rear_tz;
	transform2.rotate(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
	transform2.rotate(Eigen::AngleAxisf(rear_pitch*M_PI/180, Eigen::Vector3f::UnitY()));
	transform2.rotate(Eigen::AngleAxisf(rear_roll*M_PI/180, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*cloud_rear, *cloud_rear, transform2);


    double odom_inc = 0.0;
    double turn_inc = 0.0;
    
    cout<<setprecision(15)<<"front: "<<pcl_conversions::fromPCL(cloud_front->header.stamp).toSec()<<endl;
    cout<<setprecision(15)<<"rear: "<<pcl_conversions::fromPCL(cloud_rear->header.stamp).toSec()<<endl;
    double dt = pcl_conversions::fromPCL(cloud_rear->header.stamp).toSec() - pcl_conversions::fromPCL(cloud_front->header.stamp).toSec();
    cout<<setprecision(15)<<"devation: "<<dt<<endl;
    for(int i=0; i<odom_array.size(); i++ )
	{
		if( odom_array[i].time_stamp - odom_array[0].time_stamp < abs(dt) )
			cout<<setprecision(16)<<"odom: "<<odom_array[i].odom
				<<" "<<"time_stamp: "<<odom_array[i].time_stamp<<endl;
		else
		{
			odom_inc = odom_array[i].odom - odom_array[0].odom;
			cout<<"odom_inc: "<<odom_inc<<endl;
			break;
		}
	}
	 for(int i=1; i<turn_array.size(); i++ )
	{
		if(turn_array[i].time_stamp-turn_array[0].time_stamp < abs(dt) )
			cout<<setprecision(16)<<"turn: "<<turn_array[i].turn
				<<" "<<"time_stamp: "<<turn_array[i].time_stamp<<endl;
		else
		{
			turn_inc = turn_array[i].turn - turn_array[0].turn;
			cout<<"turn_inc: "<<turn_inc<<endl;
			break;
		}
	}
	
	double inc_x_ = odom_inc * cos( turn_inc );
	double inc_y_ = odom_inc * sin( turn_inc );
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << -inc_x_, -inc_y_, 0;
	trans.rotate(Eigen::AngleAxisf(-turn_inc, Eigen::Vector3f::UnitZ()));
	
	if(dt > 0)
		pcl::transformPointCloud(*cloud_front, *cloud_front, trans);
	else
		pcl::transformPointCloud(*cloud_rear, *cloud_rear, trans);
	
	
	cloud_front->header.frame_id = "base_link";
	cloud_rear->header.frame_id = "base_link";

	sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(*cloud_front, cloud_to_pub);
    pub_front.publish(cloud_to_pub);
    pcl::toROSMsg(*cloud_rear, cloud_to_pub);
    pub_rear.publish(cloud_to_pub);
	
	
	cout<<"clear"<<endl;
	odom_array.clear();
	turn_array.clear();
}

void configCallback(tools::lidar_calibrationConfig &config, uint32_t level)
{
	front_pitch = config.front_pitch;
	front_tx = config.front_tx;
	front_ty = config.front_ty;
	front_tz = config.front_tz;
	rear_roll = config.rear_roll;
	rear_pitch = config.rear_pitch;
	rear_yaw = config.rear_yaw;
	rear_tx = config.rear_tx;
	rear_ty = config.rear_ty;
	rear_tz = config.rear_tz;
}

void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	curr_odom += odom_inc;
//	cout<<"curr_odom: "<<curr_odom<<endl;
	OdomStamped odom_frame;
	odom_frame.odom = curr_odom;
	odom_frame.time_stamp = (double) ros::Time::now().toSec();
	odom_array.push_back(odom_frame);
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
	
	TurnStamped turn_frame;
	turn_frame.turn = curr_turn;
	turn_frame.time_stamp = turn_t;
	turn_array.push_back(turn_frame);
	
//	cout<<"dt: "<<dt<<endl;
//	cout<<"yaw_vel: "<<input->angular_velocity.z<<endl;
//	cout<<"yaw_inc: "<<yaw_inc<<endl;
//	cout<<"curr_turn: "<<curr_turn<<endl;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sync_calibration");
	ros::NodeHandle nh;
	pub_rear = nh.advertise<sensor_msgs::PointCloud2>("rear_points_in_base_link",2);
	pub_front = nh.advertise<sensor_msgs::PointCloud2>("front_points_in_base_link",2);
	
	ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, &pulseCallback);
	ros::Subscriber sub_imu = nh.subscribe("/imu_torso/xsens/data",2,&imuCallback);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_rear_cloud(nh, "/rear/velodyne_points", 2);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_front_cloud(nh, "/velodyne_points", 2);
	typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_front_cloud, sub_rear_cloud);
	sync.registerCallback(boost::bind(&synCurbCallback, _1, _2));

	ros::NodeHandle pnh("~");
	pnh.param("front_pitch", front_pitch,14.0);
	pnh.param("front_tx", front_tx,0.0);
	pnh.param("front_ty", front_ty,0.0);
	pnh.param("front_tz", front_tz,1.9);
	pnh.param("rear_roll", rear_roll,-22.0);
	pnh.param("rear_pitch", rear_pitch,0.0);
	pnh.param("rear_yaw", rear_yaw,90.0);
	pnh.param("rear_tx", rear_tx,-0.8);
	pnh.param("rear_ty", rear_ty,-0.2);
	pnh.param("rear_tz", rear_tz,1.7);



	//---------参数服务相关变量------------
	dynamic_reconfigure::Server<tools::lidar_calibrationConfig> dr_srv;
	dynamic_reconfigure::Server<tools::lidar_calibrationConfig>::CallbackType cb;
	//------配置动态更改参数服务-------
	cb = boost::bind(&configCallback, _1, _2);
	dr_srv.setCallback(cb);

	ros::spin();
	return 0;
}
