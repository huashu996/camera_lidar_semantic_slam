//
// Created by cai on 2021/8/26.
//

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include <time.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<cmath>
#include<stdio.h>
//#include "fssim_common/Cmd.h"
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
using namespace Eigen;
using namespace cv;
using namespace std;

#include "colored_pointcloud/colored_pointcloud.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

cv::Mat P_rect_00(3,4,cv::DataType<double>::type);//3×4 projection matrix after rectification
cv::Mat R_rect_00(4,4,cv::DataType<double>::type);//3×3 rectifying rotation to make image planes co-planar
cv::Mat RT(4,4,cv::DataType<double>::type);//rotation matrix and translation vector
class RsCamFusion
{
//**********************************************************************************************************
  //1、定义成员变量
  private:
    typedef     message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> slamSyncPolicy;
    message_filters::Synchronizer<slamSyncPolicy>* sync_;
    message_filters::Subscriber<sensor_msgs::Image>* camera_sub1; 
    message_filters::Subscriber<sensor_msgs::PointCloud2>* lidar_sub;
    
    pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud_toshow;
    pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud;
    pcl::PointCloud<PointXYZRGBI>::Ptr cloud_toshow;
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_toshow;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_toshow;
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud;
    cv::Mat input_image;
    cv::Mat image_to_show,image_to_show1;
    int frame_count = 0;
    static cv::Size imageSize;
    static ros::Publisher pub;
    //store calibration data in Opencv matrices

    image_transport::Publisher depth_pub ;
    sensor_msgs::ImagePtr depth_msg;
    ros::NodeHandle nh;
    ros::Publisher colored_cloud_showpub;
    ros::Subscriber sub;
    ros::Publisher fused_image_pub1;
	
  public:   
//构造函数
    RsCamFusion():

    nh("~"){

        RT.at<double>(0,0) = 7.533745e-03;RT.at<double>(0,1) = -9.999714e-01;RT.at<double>(0,2) = -6.166020e-04;RT.at<double>(0,2) = -4.069766e-03;
        RT.at<double>(1,0) = 1.480249e-02;RT.at<double>(1,1) = 7.280733e-04;RT.at<double>(1,2) = -9.998902e-01;RT.at<double>(1,3) = -7.631618e-02;
        RT.at<double>(2,0) = 9.998621e-01;RT.at<double>(2,1) = 7.523790e-03;RT.at<double>(2,2) = 1.480755e-02;RT.at<double>(2,3) = -2.717806e-01;
        RT.at<double>(3,0) = 0.0;RT.at<double>(3,1) = 0.0;RT.at<double>(3,2) = 0.0;RT.at<double>(3,3) = 1.0;

        R_rect_00.at<double>(0,0) = 9.999239e-01;R_rect_00.at<double>(0,1) = 9.837760e-03;R_rect_00.at<double>(0,2) = -7.445048e-03;R_rect_00.at<double>(0,3) = 0.0;
        R_rect_00.at<double>(1,0) = -9.869795e-03;R_rect_00.at<double>(1,1) = 9.999421e-01;R_rect_00.at<double>(1,2) = -4.278459e-03;R_rect_00.at<double>(1,3) = 0.0;
        R_rect_00.at<double>(2,0) = 7.402527e-03;R_rect_00.at<double>(2,1) = 4.351614e-03;R_rect_00.at<double>(2,2) = 9.999631e-01;R_rect_00.at<double>(2,3) = 0.0;
        R_rect_00.at<double>(3,0) = 0.0;R_rect_00.at<double>(3,1) = 0.0;R_rect_00.at<double>(3,2) = 0.0;R_rect_00.at<double>(3,3) = 1.0;

        P_rect_00.at<double>(0,0) = 7.215377e+02;P_rect_00.at<double>(0,1) = 0.000000e+00;P_rect_00.at<double>(0,2) = 6.095593e+02;P_rect_00.at<double>(0,3) = 0.000000e+00;
        P_rect_00.at<double>(1,0) = 0.000000e+00;P_rect_00.at<double>(1,1) = 7.215377e+02;P_rect_00.at<double>(1,2) = 1.728540e+02;P_rect_00.at<double>(1,3) = 0.000000e+00;
        P_rect_00.at<double>(2,0) = 0.000000e+00;P_rect_00.at<double>(2,1) = 0.000000e+00;P_rect_00.at<double>(2,2) = 1.000000e+00;P_rect_00.at<double>(2,3) = 0.000000e+00;
        camera_sub1  = new message_filters::Subscriber<sensor_msgs::Image>(nh, "/forward",300);
        lidar_sub  = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/kitti/velo/pointcloud",100);
        sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(100), *camera_sub1,*lidar_sub);
        sync_->registerCallback(boost::bind(&RsCamFusion::callback,this, _1, _2));
        cout<<"waite_image"<<endl;
        allocateMemory(); //初始化
    }

    void allocateMemory()
    {
        raw_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        colored_cloud_toshow.reset(new pcl::PointCloud<PointXYZRGBI>());
        colored_cloud.reset(new pcl::PointCloud<PointXYZRGBI>());
        cloud_toshow.reset(new pcl::PointCloud<PointXYZRGBI>());
        input_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        input_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        
    }
    void resetParameters(){ 
        raw_cloud->clear();
        input_cloud_ptr->clear();
        input_cloud->clear();
        colored_cloud_toshow->clear();
        colored_cloud->clear();
        cloud_toshow->clear();
    }
    void callback(const sensor_msgs::ImageConstPtr input_image_msg1,
            const sensor_msgs::PointCloud2ConstPtr input_cloud_msg)
    {
        resetParameters();
        cv::Mat input_image1;
        cv_bridge::CvImagePtr cv_ptr1; 
        
        std_msgs::Header image_header1 = input_image_msg1->header;
        std_msgs::Header cloud_header = input_cloud_msg->header;

        //数据获取
        //图像ROS消息转化
        
        cv_ptr1 = cv_bridge::toCvCopy(input_image_msg1,sensor_msgs::image_encodings::BGR8);
        input_image1 = cv_ptr1->image;
        
        //获取点云
        
        pcl::fromROSMsg(*input_cloud_msg, *input_cloud_ptr);//把input_cloud_msg放
        std::vector<int> indices; 
        pcl::removeNaNFromPointCloud(*input_cloud_ptr, *input_cloud_ptr, indices);//去除无效点
        transformpoint(input_cloud_ptr,input_image1,P_rect_00,R_rect_00,RT);
        cout<<"start"<<endl;
        colored_cloud_showpub = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud_toshow",10);
        publishCloudtoShow(colored_cloud_showpub, cloud_header, colored_cloud_toshow);
        fused_image_pub1 = nh.advertise<sensor_msgs::Image>("image_to_show",10);
        publishImage(fused_image_pub1, image_header1, image_to_show1);
        frame_count = frame_count + 1;
    }
         
         //××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    void transformpoint(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud, const cv::Mat input_image, cv::Mat &P_rect_00,cv::Mat &R_rect_00,cv::Mat &RT)
    {

        cv::Mat X(4,1,cv::DataType<double>::type);
        cv::Mat Y(4,1,cv::DataType<double>::type);
        cv::Point pt;
        std::vector<cv::Point3f> rawPoints;
        
        *raw_cloud = *input_cloud;
        image_to_show = input_image.clone();
        for(int i=0;i<raw_cloud->size();i++) {

            
            // convert each 3D point into homogeneous coordinates and store it in a 4D variable X
            X.at<double>(0, 0) = raw_cloud->points[i].x;
            X.at<double>(1, 0) = raw_cloud->points[i].y;
            X.at<double>(2, 0) = raw_cloud->points[i].z;
            X.at<double>(3, 0) = 1;

            //apply the projection equation to map X onto the image plane of the camera. Store the result in Y

            //计算矩阵
            Y=P_rect_00*R_rect_00*RT*X;
            pt.x=Y.at<double>(0, 0) / Y.at<double>(2, 0);
            pt.y=Y.at<double>(1, 0) / Y.at<double>(2, 0);
            // transform Y back into Euclidean coordinates and store the result in the variable pt
            float d = Y.at<double>(2, 0)*1000.0;
            float val = raw_cloud->points[i].x;
            float maxVal = 20.0;
            int red = min(255, (int) (255 * abs((val - maxVal) / maxVal)));
            int green = min(255, (int) (255 * (1 - abs((val - maxVal) / maxVal))));
            
            if(pt.x<1240 &&pt.x>0 &&pt.y<375 &&pt.y>0 &&d>0)
            {
                /*
                if (int(input_image.at<cv::Vec3b>(pt.y, pt.x)[2]) == 128 || int(input_image.at<cv::Vec3b>(pt.y, pt.x)[2]) == 152
                  ||int(input_image.at<cv::Vec3b>(pt.y, pt.x)[2]) == 107 || int(input_image.at<cv::Vec3b>(pt.y, pt.x)[2]) == 244
                  ||int(input_image.at<cv::Vec3b>(pt.y, pt.x)[2]) == 70 )
                  {
                  */
                  
                  
                    cv::circle(image_to_show, pt, 1, cv::Scalar(0, green, red), cv::FILLED);
                    image_to_show1 = image_to_show;
                    PointXYZRGBI point;
                    point.x = raw_cloud->points[i].x;   
                    point.y = raw_cloud->points[i].y;   //to create colored point clouds
                    point.z = raw_cloud->points[i].z;
                    point.intensity = raw_cloud->points[i].intensity;
                    point.g = input_image.at<cv::Vec3b>(pt.y, pt.x)[1];
                    point.b = input_image.at<cv::Vec3b>(pt.y, pt.x)[0];
                    point.r = input_image.at<cv::Vec3b>(pt.y, pt.x)[2];
                    colored_cloud->points.push_back(point); 
                    
                  

            }
            /*
            else
            {
                pcl::PointXYZRGB point;
                point.x = raw_cloud->points[i].x;   
                point.y = raw_cloud->points[i].y;   //to create colored point clouds
                point.z = raw_cloud->points[i].z;
                //point.intensity = raw_cloud->points[i].intensity;
                point.g = 0;
                point.b = 0;
                point.r = 0;
                cloud_toshow->points.push_back(point); 
            }
            */
            
          }
          *colored_cloud_toshow=*colored_cloud+*cloud_toshow;
        }

    void publishCloudtoShow(const ros::Publisher& cloudtoshow_pub, const std_msgs::Header& header,
                  const pcl::PointCloud<PointXYZRGBI>::ConstPtr& cloud)
    {
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header = header;
        cloudtoshow_pub.publish(output_msg);
    }
    void publishImage(const ros::Publisher& image_pub, const std_msgs::Header& header, const cv::Mat image)
    {
      cv_bridge::CvImage output_image;
      output_image.header = header;
      output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      output_image.image = image;
      image_pub.publish(output_image);
    } 
};
//*****************************************************************************************************
//
//                                           程序入口
//
//×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
int main(int argc, char** argv)
{
  //1、节点初始化 及定义参数
    ros::init(argc, argv, "kitti3D2");
    RsCamFusion RF;
    ros::spin();
    return 0;
}



