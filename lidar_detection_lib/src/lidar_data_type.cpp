//
// Created by wlh on 17-6-3.
//

#include <lidar_detection/lidar_data_type.h>

namespace lidar_detection
{

}

namespace lidar_detection
{
    void PointCloudRing::toRingArray()
    {
        ring_array_.clear();
        ring_array_.resize(32);
        int ring_max = 0;
        for (int i = 0; i < cloud_ring_->points.size(); ++i)
        {
            int ring_idx = cloud_ring_->points[i].ring;
            if(ring_idx>ring_max) ring_max=ring_idx;
            ring_array_[ring_idx].points.push_back(cloud_ring_->points[i]);
        }
        ring_array_.resize(ring_max+1);
    }

    pcl::PointCloud<pcl::PointXYZRGB> PointCloudRing::toColorCloud()
    {
        pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
        pcl::PointXYZRGB pt_color;
        for(auto pt : cloud_ring_->points)
        {
            if(pt.ring>=min_ring_ && pt.ring<=max_ring_)
            {
                operation::assignXYZ(pt,pt_color);
                pt_color.rgb = *reinterpret_cast<float*>(&rainbow[pt.ring%6]);
                color_cloud.push_back(pt_color);
            }
        }

        return color_cloud;
    }
}
