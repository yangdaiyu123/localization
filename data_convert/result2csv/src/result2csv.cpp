#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <fstream>

using namespace std;

ofstream output;


geometry_msgs::PoseStamped icp_pose,rtk_pose,est_pose;

double get_yaw( geometry_msgs::PoseStamped pose )
{
	double roll, pitch, yaw;
	tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

double get_xy( geometry_msgs::PoseStamped pose, double& x, double& y )
{
	x = pose.pose.position.x;
	y = pose.pose.position.y;
}

void rtkPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	rtk_pose = pose_in;
}

void icpPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	icp_pose = pose_in;
	
	double rtk_yaw = get_yaw( rtk_pose );
	double icp_yaw = get_yaw( icp_pose );
	double est_yaw = get_yaw( est_pose );
	
	double icp_dx = ( icp_pose.pose.position.x - rtk_pose.pose.position.x ) * cos( rtk_yaw );
	double icp_dy = ( icp_pose.pose.position.y - rtk_pose.pose.position.y ) * sin( rtk_yaw );
	
	output<<rtk_pose.pose.position.x<<"\t"<<rtk_pose.pose.position.y<<"\t"<<rtk_yaw<<"\t"
		<<icp_pose.pose.position.x<<"\t"<<icp_pose.pose.position.y<<"\t"<<icp_yaw<<"\t"<<icp_dx<<"\t"<<icp_dy<<"\t"
		<<est_pose.pose.position.x<<"\t"<<est_pose.pose.position.y<<"\t"<<est_yaw<<"\t"<<endl;
}

void estPoseCallback(geometry_msgs::PoseStamped pose_in)
{
	est_pose = pose_in;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose2path");
	
	ros::NodeHandle nh;

	ros::Subscriber sub_rtk_pose = nh.subscribe("rtk_pose", 2, rtkPoseCallback);
	ros::Subscriber sub_icp_pose = nh.subscribe("icp_pose", 2, icpPoseCallback);
	ros::Subscriber sub_est_pose = nh.subscribe("estimate_pose", 2, estPoseCallback);
	
	output.open ("example.txt", std::ios_base::app);
	if (output.is_open())
		output<<"rtk_x\trtk_y\trtk_yaw\ticp_x\ticp_y\ticp_yaw\ticp_dx\ticp_dy\test_x\test_y\test_yaw"<<endl;
	

	
	ros::spin();
	return 0;
}

