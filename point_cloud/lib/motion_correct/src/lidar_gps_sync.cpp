#include "ros/ros.h"
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cloud_operation.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"


//0.0210386;
const double ODOMETRY_FACTOR = 0.0210386;

ros::Publisher pub_scan;
ros::Publisher pub_correct;
ros::Publisher pub_imu_pitch;

double curr_odom = 0.0, last_odom = 0.0;
bool rece_imu = false;
bool is_inited = false;
double turn_t = 0.0;
double curr_turn = 0.0, last_turn = 0.0;
double cloud_t = 0.0, last_cloud_t = 0.0;
double pose_x=0.0, pose_y=0.0, last_pose_x=0.0, last_pose_y=0.0;
double imu_odom=0.0;
double imu_pitch=0.0;
std::vector<int> fifo_size;
pcl::PointCloud<pcl::PointXYZI> cloud_sum;
geometry_msgs::PoseStamped gps_pose;


template <typename PointT>
void cloud_transform(pcl::PointCloud<PointT>& cloud, double odom_inc, double yaw_inc)
{
    double inc_x = odom_inc * cos(yaw_inc);
    double inc_y = odom_inc * sin(yaw_inc);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -inc_x, -inc_y, 0.0;
    transform.rotate(Eigen::AngleAxisf(-yaw_inc, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(cloud, cloud, transform);
}

template <typename PointT>
void transform_baselink_to_map(pcl::PointCloud<PointT>& cloud, geometry_msgs::PoseStamped gps_pose)
{
    double roll, pitch, yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(gps_pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double x = gps_pose.pose.position.x;
    double y = gps_pose.pose.position.y;
    double z = gps_pose.pose.position.z;

    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation() << x, y, z;
    trans.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));

    pcl::transformPointCloud(cloud, cloud, trans);
}


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{

    pcl::PointCloud<pcl::PointXYZI> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);
    double time_diff = (double)gps_pose.header.stamp.toSec() - (double)msg->header.stamp.toSec();
    std::cout<<"lidar time stamp: "<<msg->header.stamp<<std::endl;
    std::cout<<"gps time stamp:   "<<gps_pose.header.stamp<<std::endl;
    std::cout<<"time difference:  "<<time_diff<<std::endl;
    std::cout<<"odom increase:    "<<imu_odom<<std::endl;
    std::cout<<"dist interploration: "<<imu_odom/(0.1)*time_diff<<std::endl;

    cloud_transform(cloud_in, imu_odom/(0.1)*time_diff, 0.0);
    transform_baselink_to_map(cloud_in,gps_pose);
    cloud_sum += cloud_in;

//    cloud_sum.header.frame_id = "map";
    sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(cloud_sum,cloud_to_pub);
    cloud_to_pub.header.stamp = msg->header.stamp;
    cloud_to_pub.header.frame_id = "map";
    pub_correct.publish(cloud_to_pub);

    if(curr_odom>100.0)
    {
        pcl::io::savePCDFileASCII ("/home/wlh/map/lidar_gps_sync/pole_map.pcd", cloud_sum);
        std::cout<<"create map"<<std::endl;
    }

}

void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
    int pulse_inc = (int)input->data[0] + (int)input->data[1];
    float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
    curr_odom += odom_inc;
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
    double yaw_inc = (double) (input->angular_velocity.z) * dt;
    curr_turn += yaw_inc;

    double pitch_inc = (double) (input->angular_velocity.y) * dt;
    imu_pitch += pitch_inc;
    std_msgs::Float64 pitch_deg;
    pitch_deg.data = imu_pitch * 180.0 / M_PI;
    pub_imu_pitch.publish(pitch_deg);

    turn_t = (double) ros::Time::now().toSec();
}

void rtkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    gps_pose = *input;

    pose_x = input->pose.position.x;
    pose_y = input->pose.position.y;

    double dist = std::sqrt( std::pow(pose_x-last_pose_x, 2) + std::pow(pose_y-last_pose_y, 2) );
    imu_odom = dist;
//    std::cout<<"x:"<<input->pose.position.x<<"\ty: "<<input->pose.position.y<<"\tdist: "<<dist<<"\timu_odom: "<<imu_odom<<std::endl;

    last_pose_x = pose_x;
    last_pose_y = pose_y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_gps_sync");
    ros::NodeHandle nh;
    ros::Subscriber sub_front_curb = nh.subscribe("stixel_cloud",2,pointCloudCallback);
    ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, pulseCallback);
    ros::Subscriber sub_imu = nh.subscribe("imu_torso/xsens/data", 2, imuCallback);
    ros::Subscriber sub_rtk = nh.subscribe("fusion_pose", 2, rtkPoseCallback);
    pub_correct = nh.advertise<sensor_msgs::PointCloud2>("pole_map", 2);

    cloud_sum.clear();
    fifo_size.clear();

    ros::spin();
    return 0;
}