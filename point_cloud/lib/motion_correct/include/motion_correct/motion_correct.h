#ifndef _MOTION_CORRECT_H_
#define _MOTION_CORRECT_H_

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>



class MotionCorrect
{
public:
    void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input);
    void setMotionInfo(const float odom_inc, const float yaw_inc, const float time_inc);
    void getCorrectedCloud(pcl::PointCloud<pcl::PointXYZI>& output);


private:
    float odom_inc_;
    float yaw_inc_;
    float time_inc_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_;
    pcl::PointCloud<pcl::PointXYZI> output_;


};




#endif