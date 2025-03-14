#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


struct EIGEN_ALIGN16 PointXYZRGBITNormalTagL {
      PCL_ADD_POINT4D;
      PCL_ADD_NORMAL4D;
      PCL_ADD_RGB;
      float intensity;
      float curvature;
      uint32_t t;
      uint8_t tag;
      uint8_t line;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBITNormalTagL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, rgb, rgb)
    (float, intensity, intensity)
    (float, curvature, curvature)
    (std::uint32_t, t, t)
    (std::uint8_t, tag, tag)
    (std::uint8_t, line, line)

)

struct EIGEN_ALIGN16 PointXYZITTagL {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint8_t tag;
      uint8_t line;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };


POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITTagL,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint8_t, tag, tag)
    (std::uint8_t, line, line)

)

#endif //PCL_NO_PRECOMPILE


