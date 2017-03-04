#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

#include "geo_pos_conv.hh"

using namespace std;

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;

static double yaw;
static double yaw_filtered;

string topic_pose, topic_gps, topic_yaw;
double origin_lat, origin_lon;


void gpsFilteredCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
	geo_pos_conv geo;



    geo.set_plane(origin_lat*M_PI/180, origin_lon*M_PI/180);
    geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

    static tf::TransformBroadcaster rtk_fusion_broadcaster;
    tf::Transform rtk_fusion_transform;
    tf::Quaternion pose_q;

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = geo.y();
    pose.pose.position.y = geo.x();
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_filtered);
    pose_publisher.publish(pose);

    static tf::TransformBroadcaster br_filtered;
    tf::Transform transform_filtered;
    tf::Quaternion q_filtered;

    transform_filtered.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y,pose.pose.position.z));
    q_filtered.setRPY(0, 0, yaw_filtered);
    transform_filtered.setRotation(q_filtered);
    br_filtered.sendTransform( tf::StampedTransform(transform_filtered, msg->header.stamp, "map","base_link"));
}

void yawFilteredCallback(const std_msgs::Float64::ConstPtr& yaw_rad_in)
{
	yaw_filtered = yaw_rad_in->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fix2pose");
    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");
    pnh.param<std::string>("topic_pose", topic_pose, "gps_pose");
    pnh.param<std::string>("topic_gps", topic_gps, "gps_filtered");
    pnh.param<std::string>("topic_yaw", topic_yaw, "yaw_filtered");
    pnh.param("origin_lat", origin_lat, 31.0);
    pnh.param("origin_lon", origin_lon, 121.0);

    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>(topic_pose,10);
    stat_publisher = nh.advertise<std_msgs::Bool>("/gps_stat", 10);
    ros::Subscriber sub_rtk_filtered = nh.subscribe(topic_gps, 10, gpsFilteredCallback);
    ros::Subscriber sub_yaw_filtered = nh.subscribe(topic_yaw, 10, yawFilteredCallback);

    ros::spin();
    return 0;
}
