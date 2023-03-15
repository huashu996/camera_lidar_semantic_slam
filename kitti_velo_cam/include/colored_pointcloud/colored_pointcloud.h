#include <pcl/point_types.h>
 
struct EIGEN_ALIGN16 PointXYZRGBI
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    float intensity;     //intensity
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBI,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (uint8_t,rgb,rgb)
                                  (float,intensity,intensity)
)
