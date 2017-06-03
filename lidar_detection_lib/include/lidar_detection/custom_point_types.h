#ifndef _CUSTOM_POINT_TYPES_H_
#define _CUSTOM_POINT_TYPES_H_

/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
	PCL_ADD_POINT4D;                    // quad-word XYZ
	float    intensity;                 ///< laser intensity reading
	uint16_t ring;                      ///< laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

typedef PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

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


namespace
{
  // RGB color values
  const int color_red =       0xff0000;
  const int color_orange =    0xff8800;
  const int color_yellow =    0xffff00;
  const int color_green =     0x00ff00;
  const int color_blue =      0x0000ff;
  const int color_violet =    0xff00ff;

  const int N_COLORS = 6;
  int rainbow[N_COLORS] = {color_red, color_orange, color_yellow,
                           color_green, color_blue, color_violet};
}




#endif
