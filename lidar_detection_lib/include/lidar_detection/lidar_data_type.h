#ifndef _LIDAR_DATA_TYPE_H_
#define _LIDAR_DATA_TYPE_H_

#include <lidar_detection/common_headers.h>
#include <lidar_detection/basic_operate.h>

namespace lidar_detection
{

    template <typename PointT>
    class PointCloudArray
    {
    public:
        static std::vector<pcl::PointCloud<PointT> > get_sub_array(std::vector<pcl::PointCloud<PointT> > input, int begin, int end)
        {
            if(begin < 0 || end > input.size()-1) std::cout<<"error: function get_sub_array, out of index!"<<std::endl;
            std::vector<pcl::PointCloud<PointT> > sub_array;
            for(int i=begin; i<end; i++)
                sub_array.push_back(input[i]);
            return sub_array;
        }
    };

    class PointCloudRing
    {

    public:
        PointCloudRing()
        {
            min_ring_ = 0;
            max_ring_ = 4;
            ring_array_.clear();
        }

        void setInputCloud(pcl::PointCloud<PointXYZIR>::Ptr input)
        {
            cloud_ring_ = input;
        }

        pcl::PointCloud<PointXYZIR> getCloudRing()
        {
            return *cloud_ring_;
        }

        void setMinMaxRing(const int min, const int max)
        {
            min_ring_ = min;
            max_ring_ = max;
        }

        void toRingArray();

        pcl::PointCloud<pcl::PointXYZRGB> toColorCloud();

    private:
        pcl::PointCloud<PointXYZIR>::Ptr cloud_ring_;
        std::vector<pcl::PointCloud<PointXYZIR> > ring_array_;

        int max_ring_;
        int min_ring_;

    };

}


#endif