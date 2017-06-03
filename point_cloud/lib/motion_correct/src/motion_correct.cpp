#include "motion_correct.h"

void MotionCorrect::setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
    input_ = input;
}

void MotionCorrect::setMotionInfo(const float odom_inc, const float yaw_inc, const float time_inc)
{

}

void MotionCorrect::getCorrectedCloud(pcl::PointCloud<pcl::PointXYZI> &output)
{

}

