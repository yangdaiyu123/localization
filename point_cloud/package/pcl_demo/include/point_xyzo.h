#ifndef _POINT_XYZO_H_
#define _POINT_XYZO_H_

struct PointXYZO
{
	PCL_ADD_POINT4D;
	float    orientation;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZO,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, orientation, orientation))

#endif                                  
