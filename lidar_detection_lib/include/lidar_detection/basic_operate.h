//
// Created by wlh on 17-6-3.
//

#ifndef _BASIC_OPERATE_H_
#define _BASIC_OPERATE_H_

#include <lidar_detection/common_headers.h>

namespace lidar_detection
{
    namespace operation
    {
        template <typename PointT1, typename PointT2> void assignXYZ(PointT1 src, PointT2 dst)
        {
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
        }

    }
}

#endif
