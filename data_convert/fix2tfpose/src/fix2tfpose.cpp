/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

static ros::Publisher pose_publisher;

static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
static double yaw_filtered;

static void GNSSCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
    geo_pos_conv geo;

    geo.set_plane(31.0*M_PI/180, 121.0*M_PI/180);
    geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

    static tf::TransformBroadcaster pose_broadcaster;
    tf::Transform pose_transform;
    tf::Quaternion pose_q;

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
   // pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = geo.y();
    pose.pose.position.y = geo.x();
    pose.pose.position.z = geo.z();

    // set gnss_stat
    if(pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0){
      gnss_stat_msg.data = false;
    } else{
      gnss_stat_msg.data = true;
    }

    double distance = sqrt(
            pow(pose.pose.position.y - _prev_pose.pose.position.y, 2)
                    + pow(pose.pose.position.x - _prev_pose.pose.position.x,
                            2));
    std::cout << "distance : " << distance << std::endl;

    if (distance > 5) {
        yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y,
                pose.pose.position.x - _prev_pose.pose.position.x);
        _quat = tf::createQuaternionMsgFromYaw(yaw);
        _prev_pose = pose;
    }

    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
    stat_publisher.publish(gnss_stat_msg);

    //座標変換
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;

    transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y,pose.pose.position.z));
    q.setRPY(0, 0, yaw);
    transform.setRotation(q);
//    br.sendTransform( tf::StampedTransform(transform, msg->header.stamp, "map","base_link"));
}

void gpsFilteredCallback(const sensor_msgs::NavSatFixConstPtr &msg)
{
	geo_pos_conv geo;

    geo.set_plane(31.0*M_PI/180, 121.0*M_PI/180);
    geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

    static tf::TransformBroadcaster rtk_fusion_broadcaster;
    tf::Transform rtk_fusion_transform;
    tf::Quaternion pose_q;

    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
   // pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = geo.y();
    pose.pose.position.y = geo.x();
    pose.pose.position.z = geo.z();
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
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("gps_pose",10);
    stat_publisher = nh.advertise<std_msgs::Bool>("/gps_stat", 10);
//    ros::Subscriber sub_gps_raw = nh.subscribe("/rtk/fix", 10,GNSSCallback);
    ros::Subscriber sub_rtk_filtered = nh.subscribe("/gps_filtered", 10,gpsFilteredCallback);
    ros::Subscriber sub_yaw_filtered = nh.subscribe("/yaw_filtered", 10,yawFilteredCallback);

    ros::spin();
    return 0;
}
