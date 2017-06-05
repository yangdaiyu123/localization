#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <lidar_detection/common_headers.h>
#include <lidar_detection/lidar_detection.h>
#include <lidar_detection/lidar_data_type.h>

ros::Publisher pub_cloud;
//pcl::visualization::CloudViewer viewer("cloud_viewer");

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<PointXYZIR> cloud_ring;
    pcl::fromROSMsg(*msg, cloud_ring);
    lidar_detection::PointCloudRing clr;
    clr.setInputCloud(cloud_ring.makeShared());
    clr.setMinMaxRing(0,3);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_color = clr.toColorCloud();

    cloud_color.header.frame_id="base_link";
    sensor_msgs::PointCloud2 cloud_to_pub;
    pcl::toROSMsg(cloud_color,cloud_to_pub);
    pub_cloud.publish(cloud_to_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_detection_test");

    ros::NodeHandle nh;
    ros::Subscriber sub_cloud = nh.subscribe("points_in_base_link",2,&pointCloudCallback);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("color_cloud",2);

    ros::spin();

    return 0;
}
