#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
//操作自定义的点云需要这个头文件
#define PCL_NO_PRECOMPILE
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "cloud_msgs/cloud_info.h"

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;



// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

/*
extern const int N_SCAN =40;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.33;
extern const float ang_bottom = 3.0+0.1;
extern const int groundScanInd = 10;
*/
/*
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 20;
*/
//默认回环检测关闭:
extern const bool loopClosureEnableFlag = false;
//建图处理时间间隔，只有时间间隔大于这个值才能进行建图优化：
extern const double mappingProcessInterval = 0.3;

//处理频率0.1s
extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
//iMU队列长
extern const int imuQueLength = 200;
//imu 话题名
extern const string pointCloudTopic = "/colored_cloud_toshow";
extern const string imuTopic = "/imu/data";


extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 1.0472; // segmentTheta=1.0472<==>60度,在imageProjection中用于判断平面
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 6;
extern const int surfFeatureNum = 10;
extern const int sectionsTotal = 2;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

extern const float surroundingKeyframeSearchRadius = 30.0;
extern const int   surroundingKeyframeSearchNum = 50;

extern const float historyKeyframeSearchRadius = 5.0;
extern const int   historyKeyframeSearchNum = 25;
extern const float historyKeyframeFitnessScore = 0.3;

extern const float globalMapVisualizationSearchRadius = 1500.0;


struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};
 
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
typedef PointXYZRGBI  PointType;
//×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
struct PointXYZIRPYRGBT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    PCL_ADD_RGB;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYRGBT,
                                   (float, x, x) 
                                   (float, y, y)
                                   (float, z, z) 
                                   (float, intensity, intensity)
                                   (float, roll, roll) 
                                   (float, pitch, pitch) 
                                   (float, yaw, yaw)
                                   (double, time, time)
                                   (uint8_t,rgb,rgb)
)

typedef PointXYZIRPYRGBT  PointTypePose;

#endif
