//
// Created by wlh on 17-6-4.
//

#ifndef LIDAR_DETECTION_BASIC_MATH_H
#define LIDAR_DETECTION_BASIC_MATH_H

#include <cmath>

namespace lidar_detection
{
    namespace math
    {
        template<typename PointT>
        double cal_angle(const PointT pt)
        {
            double angle = atan2(pt.y, pt.x)*180/M_PI;
            angle = angle < 0 ? angle += 360 : angle;
            return angle;
        }


    }
}

#endif //LIDAR_DETECTION_BASIC_MATH_H
