#ifndef _LIDAR_DATA_TYPE_H_
#define _LIDAR_DATA_TYPE_H_

#include <lidar_detection/common_headers.h>
#include <lidar_detection/basic_operate.h>
#include "basic_math.h"

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
        PointCloudRing():min_ring_(0),max_ring_(4)
        {
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

        pcl::PointCloud<PointXYZIR> getOneRing(int idx);

        pcl::PointCloud<pcl::PointXYZRGB> toColorCloud();

    private:
        pcl::PointCloud<PointXYZIR>::Ptr cloud_ring_;
        std::vector<pcl::PointCloud<PointXYZIR> > ring_array_;

        int max_ring_;
        int min_ring_;

    };

    template <typename PointT>
    class PointCloudStixel
    {
    public:
        PointCloudStixel(int w, int h, int sc):width_(w), height_(h), scale_(sc)
        {
            stixel_array_.clear();
        }

        PointCloudStixel(float x_min, float x_max, float y_min, float y_max, float sc):
                cloud_x_min_(x_min), cloud_x_max_(x_max), cloud_y_min_(y_min), cloud_y_max_(y_max), scale_(sc)
        {
            width_ = (cloud_y_max_ - cloud_y_min_)/scale_;
            height_= (cloud_x_max_ - cloud_x_min_)/scale_;
            stixel_array_.clear();
            stixel_array_.resize(height_);
            for (int i = 0; i < height_; ++i)
                stixel_array_[i].resize(width_);

//            std::cout<<"max_h: "<<height_<<"\tmax_w: "<<width_<<std::endl;
//            std::cout<<"rows: "<<stixel_array_.size()<<"\tcols: "<<stixel_array_[0].size()<<std::endl;
        }

        void setInputCloud(typename pcl::PointCloud<PointT>::Ptr input)
        {
            cloud_stixel_ = input;
        }

        pcl::PointCloud<PointT> getCloudStixel()
        {
            return *cloud_stixel_;
        }

        static std::vector<std::vector<pcl::PointCloud<PointT> > >
        cloudToStixel(typename pcl::PointCloud<PointT>::Ptr cloud, float x_min, float x_max, float y_min, float y_max,
                                                float sc)
        {
            std::vector<std::vector<pcl::PointCloud<PointT> > > stixel;
            int max_h = (x_max-x_min)/sc;
            int max_w = (y_max-y_min)/sc;
            stixel.clear();
            stixel.resize(max_h);
            for(int i=0; i<max_h; i++)
                stixel[i].resize(max_w);

//            std::cout<<"max_h: "<<max_h<<"\tmax_w: "<<max_w<<std::endl;
//            std::cout<<"rows: "<<stixel.size()<<"\tcols: "<<stixel[0].size()<<std::endl;

            for(auto pt:cloud->points)
            {
                int colum = int( (y_max - pt.y) / sc);
                int row = int( (x_max - pt.x) / sc );
                if(colum<0||colum>=max_w) continue;
                if(row<0||row>=max_h) continue;

                stixel[row][colum].points.push_back(pt);
            }

            return stixel;
        }

        std::vector<std::vector<pcl::PointCloud<PointT> > > cloudToStixel()
        {
            for(auto pt:cloud_stixel_->points)
            {
                int colum = int( (cloud_y_max_ - pt.y) / scale_);
                int row = int( (cloud_x_max_ - pt.x) / scale_ );
                if(colum<0||colum>=width_) continue;
                if(row<0||row>=height_) continue;

                stixel_array_[row][colum].points.push_back(pt);
            }
            return stixel_array_;
        }

        static pcl::PointCloud<PointT>
        stixelToCloud(std::vector<std::vector<pcl::PointCloud<PointT> > > stixel)
        {
            pcl::PointCloud<PointT> cloud;
            cloud.clear();
            for(int i=0; i<stixel.size(); i++)
                for(int j=0; j<stixel[i].size(); j++)
                {
                    cloud+=stixel[i][j];
                }
            return cloud;
        }

        pcl::PointCloud<PointT> stixelToCloud()
        {
            return stixelToCloud(stixel_array_);
        }

        pcl::PointCloud<PointT> at(int i, int j)
        {
            return stixel_array_[i][j];
        }

    private:
        typename pcl::PointCloud<PointT>::Ptr cloud_stixel_;
        std::vector<std::vector<pcl::PointCloud<PointT> > > stixel_array_;
        int width_;
        int height_;
        float scale_;
        float cloud_x_min_;
        float cloud_x_max_;
        float cloud_y_min_;
        float cloud_y_max_;
    };



    template <typename PointT>
    class CircularGrid
    {
    public:
        void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr input)
        {
            input_ = input;
        }

        std::vector<pcl::PointCloud<PointT> > toCirArray()
        {
            cir_array_.clear();
            cir_array_.resize(500);
            if(input_->points.empty())
            {
                std::cout<<"warning: function to_cir_array() input empty cloud"<<std::endl;
                return cir_array_;
            }

            for(auto pt:input_->points)
            {
                int ang_idx = math::cal_angle(pt);
                cir_array_[ang_idx].push_back(pt);
            }

            cir_array_.resize(360);
            return cir_array_;
        }

        std::vector<pcl::PointCloud<PointT> > toCirArray2()
        {
            cir_array_.clear();
            cir_array_.resize(500);
            if(input_->points.empty())
            {
                std::cout<<"warning: function to_cir_array() input empty cloud"<<std::endl;
                return cir_array_;
            }

            int ang_offset = math::cal_angle(input_->points[0]);
            int max_ang_idx = 360;

            for(int i=0; i<input_->points.size(); i++ )
            {
                int ang_idx = math::cal_angle(input_->points[i]);
                int _offset = ang_idx - ang_offset;
                int plus_360;
                if(_offset < 0) plus_360 = _offset + 360;
                else plus_360 = _offset;
                int reverse = plus_360 ? 360 - plus_360 : plus_360;
                if(i>input_->points.size()/2 && reverse < 90) reverse+=360;
                if(max_ang_idx < reverse) max_ang_idx = reverse;

                cir_array_[reverse].push_back(input_->points[i]);
            }

            cir_array_.resize(360);
            return cir_array_;
        }

        pcl::PointCloud<pcl::PointXYZRGB> showColorCloud(int begin, int end)
        {
            pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
            color_cloud.clear();

            long int color_section = 0x000fff - 0x000000;
            long int angle_section = 360;
            double rate = color_section/angle_section;

            std::cout<<"color_section: "<<color_section<<"\tangle_section: "<<angle_section<<"\trate: "<<rate<<std::endl;

            for (int i = begin; i < end; ++i)
            {
                pcl::PointXYZRGB pt;
                long int color = rate * i ;
                std::cout<<"color: "<<color<<std::endl;
                pt.r = color_table[i*3+0];
                pt.g = color_table[i*3+1];
                pt.b = color_table[i*3+2];
                if(cir_array_[i].points.empty()) continue;
                for(auto po:cir_array_[i].points)
                {
                    operation::assignXYZ(po,pt);
                    color_cloud.points.push_back(pt);
                }
            }
            return color_cloud;
        }


//        std::vector<pcl::PointCloud<PointT> > toCirArray(std::vector<pcl::PointCloud<PointT> > input)
//        {
//            std::vector<std::vector<pcl::PointCloud<PointT> > > temp_array;
//            std::vector<pcl::PointCloud<PointT> > ret_array;
//            int max_size = 360;
//
//            for(int i=0; i<input.size(); i++)
//            {
//                cir_array = toCirArray(input[i]);
//                if(max_size<cir_array.size()) max_size = cir_array.size();
//                temp_array.push_back(cir_array);
//                std::cout<<"i: "<<i<<" size: "<<cir_array.size()<<std::endl;
//            }
//            ret_array.resize(max_size);
//            for(int i=0; i<temp_array.size(); i++)
//                for(int j=0; j<temp_array[i].size(); j++)
//                    ret_array[j] += temp_array[i][j];
//            return ret_array;
//        }

//        pcl::PointCloud<pcl::PointXYZRGB> getColorCloud()
//        {
//            pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
//            for (int i = 0; i < cir_array.size()-1; ++i)
//            {
//                pcl::PointXYZRGB pt;
//                std::vector<pcl::PointCloud<PointT> > sub_array = get_sub_array(cir_array,i,i+1);
//                pcl::PointCloud<pcl::PointXYZRGB> color = arrayToColorCloud(sub_array,i%6);
//                color_cloud += color;
//            }
//            return color_cloud;
//        }
//


    private:
        typename pcl::PointCloud<PointT>::Ptr input_;
        std::vector<pcl::PointCloud<PointT> > cir_array_;



    };


}


#endif