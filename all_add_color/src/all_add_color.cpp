#include <ros/ros.h>
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <math.h>
#include "colored_pointcloud/colored_pointcloud.h"
#include <sys/stat.h>
#include <sys/types.h> 
#include <cstdio>
#include <ctime>

#define YELLOW "\033[33m" /* Yellow */
#define GREEN "\033[32m"  /* Green */
#define REND "\033[0m" << std::endl

#define WARN (std::cout << YELLOW)
#define INFO (std::cout << GREEN)
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


// colored_cloud_pub;
class RsCamFusion
{
//**********************************************************************************************************
  //1、定义成员变量
  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::PointCloud2> slamSyncPolicy;
    message_filters::Synchronizer<slamSyncPolicy>* sync_;
    message_filters::Subscriber<sensor_msgs::Image>* camera_sub1; 
    message_filters::Subscriber<sensor_msgs::Image>* camera_sub2; 
    message_filters::Subscriber<sensor_msgs::Image>* camera_sub3; 
    message_filters::Subscriber<sensor_msgs::Image>* camera_sub4; 
    message_filters::Subscriber<sensor_msgs::PointCloud2>* lidar_sub; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_points;
    
    pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud;
    pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud_toshow;
    pcl::PointCloud<PointXYZRGBI>::Ptr colored_cloud_transback;
    
    std::vector<cv::Point2d> imagePoints;
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh;
    ros::Publisher fused_image_pub;
    ros::Publisher fused_image_pub1;
    ros::Publisher fused_image_pub2;
    ros::Publisher fused_image_pub3;
    ros::Publisher fused_image_pub4;
    ros::Publisher colored_cloud_showpub;
    ros::Publisher point_cloud;
    cv::Mat intrinsic1;
    cv::Mat intrinsic2;
    cv::Mat intrinsic3;
    cv::Mat intrinsic4;
    cv::Mat extrinsic1;
    cv::Mat extrinsic2;
    cv::Mat extrinsic3;
    cv::Mat extrinsic4;
    cv::Mat distcoeff1;
    cv::Mat distcoeff2;
    cv::Mat distcoeff3;
    cv::Mat distcoeff4;
    cv::Size imageSize;
    cv::Mat input_image;
    cv::Mat image_to_show;
    cv::Mat image_to_show1,image_to_show2,image_to_show3,image_to_show4,all_image;
    Eigen::Matrix4d transform1;
    Eigen::Matrix4d transform2;
    Eigen::Matrix4d transform3;
    Eigen::Matrix4d transform4;
    Eigen::Matrix4d inv_transform1;
    Eigen::Matrix4d inv_transform2;
    Eigen::Matrix4d inv_transform3;
    Eigen::Matrix4d inv_transform4;
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // Rotation vector
    cv::Mat rMat = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // Translation vector
    bool show_colored_cloud;
    int color[21][3] = 
    {
        {255, 0, 0}, {255, 69, 0}, {255, 99, 71}, 
        {255, 140, 0}, {255, 165, 0}, {238, 173, 14},
        {255, 193, 37}, {255, 255, 0}, {255, 236, 139},
        {202, 255, 112}, {0, 255, 0}, {84, 255, 159},
        {127, 255, 212}, {0, 229, 238}, {152, 245, 255},
        {178, 223, 238}, {126, 192, 238}, {28, 134, 238},
        {0, 0, 255}, {72, 118, 255}, {122, 103, 238} 
    };
    std::string image_save_dir, cloud_save_dir, colored_cloud_save_dir;
    float color_distance;   //step length to color the lidar points according to plane distance(z)
    int frame_count = 0;
    std::string config_path, file_name;
    std::string camera_topic1,camera_topic2,camera_topic3,camera_topic4,lidar_topic;
    float color_dis;
    bool show_cloud, save_data;
    int m;
    pcl::PointCloud<pcl::PointXYZI>::Ptr input;

  public:
//**********************************************************************************************************
  //2、定义构造函数   传入参数
    RsCamFusion():

    priv_nh("~"){
      
      //从launch引入参数

      if (priv_nh.hasParam("calib_file_path") && priv_nh.hasParam("file_name"))
      {
        priv_nh.getParam("camera_topic1", camera_topic1);
        priv_nh.getParam("camera_topic2", camera_topic2);
        priv_nh.getParam("camera_topic3", camera_topic3);
        priv_nh.getParam("camera_topic4", camera_topic4);
        priv_nh.getParam("lidar_topic", lidar_topic);
        priv_nh.getParam("calib_file_path", config_path);
        priv_nh.getParam("file_name", file_name);
        priv_nh.getParam("color_distance", color_dis);
        priv_nh.getParam("show_colored_cloud", show_cloud);
        priv_nh.getParam("save_data", save_data);
      }
      else
      {
        WARN << "Config file is empty!" << REND;
        //return 0;在构造函数里不能返回值
      }
      
      INFO << "config path: " << config_path << REND;
      INFO << "config file: " << file_name << REND;
      std::string config_file_name = config_path + "/" + file_name;
      cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
      cv::Mat cam_intrinsic1,cam_intrinsic2,cam_intrinsic3,cam_intrinsic4,
      lidar2cam_extrinsic1,lidar2cam_extrinsic2,lidar2cam_extrinsic3,lidar2cam_extrinsic4, 
      cam_distcoeff1,cam_distcoeff2,cam_distcoeff3,cam_distcoeff4;
      cv::Size img_size;
      fs_reader["CameraMat1"] >> cam_intrinsic1;
      fs_reader["CameraMat2"] >> cam_intrinsic2;
      fs_reader["CameraMat3"] >> cam_intrinsic3;
      fs_reader["CameraMat4"] >> cam_intrinsic4;
      fs_reader["CameraExtrinsicMat1"] >> lidar2cam_extrinsic1;
      fs_reader["CameraExtrinsicMat2"] >> lidar2cam_extrinsic2;
      fs_reader["CameraExtrinsicMat3"] >> lidar2cam_extrinsic3;
      fs_reader["CameraExtrinsicMat4"] >> lidar2cam_extrinsic4;
      fs_reader["DistCoeff1"] >> cam_distcoeff1;
      fs_reader["DistCoeff2"] >> cam_distcoeff2;
      fs_reader["DistCoeff3"] >> cam_distcoeff3;
      fs_reader["DistCoeff4"] >> cam_distcoeff4;
      fs_reader["ImageSize"] >> img_size;
      fs_reader.release();
      //2、加入雷达和相机的数据|| camera_topic2.empty()|| camera_topic3.empty()|| camera_topic4.empty()
      if (lidar_topic.empty() || camera_topic1.empty())
      {
        WARN << "sensor topic is empty!" << REND;
        //return 0;
      }
      INFO << "lidar topic: " << lidar_topic << REND;//雷达数据
      INFO << "camera topic1: " << camera_topic1 << REND;//相机数据
      INFO << "camera topic2: " << camera_topic2 << REND;//相机数据
      INFO << "camera topic3: " << camera_topic3 << REND;//相机数据
      INFO << "camera topic4: " << camera_topic4 << REND;//相机数据
      INFO << "camera intrinsic matrix1: " << cam_intrinsic1 << REND;//相机内参矩阵
      INFO << "camera intrinsic matrix2: " << cam_intrinsic2 << REND;//相机内参矩阵
      INFO << "camera intrinsic matrix3: " << cam_intrinsic3 << REND;//相机内参矩阵
      INFO << "camera intrinsic matrix4: " << cam_intrinsic4 << REND;//相机内参矩阵
      intrinsic1 = cam_intrinsic1;//内参矩阵
      intrinsic2 = cam_intrinsic2;//内参矩阵
      intrinsic3 = cam_intrinsic3;//内参矩阵
      intrinsic4 = cam_intrinsic4;//内参矩阵
      extrinsic1 = lidar2cam_extrinsic1;//外参矩阵
      extrinsic2 = lidar2cam_extrinsic2;//外参矩阵
      extrinsic3 = lidar2cam_extrinsic3;//外参矩阵
      extrinsic4 = lidar2cam_extrinsic4;//外参矩阵
      distcoeff1 = cam_distcoeff1;
      distcoeff2 = cam_distcoeff2;
      distcoeff3 = cam_distcoeff3;
      distcoeff4 = cam_distcoeff4;
      //传入外参
      transform1(0,0) = extrinsic1.at<double>(0,0);
      transform1(0,1) = extrinsic1.at<double>(0,1);
      transform1(0,2) = extrinsic1.at<double>(0,2);
      transform1(0,3) = extrinsic1.at<double>(0,3);
      transform1(1,0) = extrinsic1.at<double>(1,0);
      transform1(1,1) = extrinsic1.at<double>(1,1);
      transform1(1,2) = extrinsic1.at<double>(1,2);
      transform1(1,3) = extrinsic1.at<double>(1,3);
      transform1(2,0) = extrinsic1.at<double>(2,0);
      transform1(2,1) = extrinsic1.at<double>(2,1);
      transform1(2,2) = extrinsic1.at<double>(2,2);
      transform1(2,3) = extrinsic1.at<double>(2,3);
      transform1(3,0) = extrinsic1.at<double>(3,0);
      transform1(3,1) = extrinsic1.at<double>(3,1);
      transform1(3,2) = extrinsic1.at<double>(3,2);
      transform1(3,3) = extrinsic1.at<double>(3,3);
      inv_transform1 = transform1.inverse(); //逆序
      
      //传入外参
      transform2(0,0) = extrinsic2.at<double>(0,0);
      transform2(0,1) = extrinsic2.at<double>(0,1);
      transform2(0,2) = extrinsic2.at<double>(0,2);
      transform2(0,3) = extrinsic2.at<double>(0,3);
      transform2(1,0) = extrinsic2.at<double>(1,0);
      transform2(1,1) = extrinsic2.at<double>(1,1);
      transform2(1,2) = extrinsic2.at<double>(1,2);
      transform2(1,3) = extrinsic2.at<double>(1,3);
      transform2(2,0) = extrinsic2.at<double>(2,0);
      transform2(2,1) = extrinsic2.at<double>(2,1);
      transform2(2,2) = extrinsic2.at<double>(2,2);
      transform2(2,3) = extrinsic2.at<double>(2,3);
      transform2(3,0) = extrinsic2.at<double>(3,0);
      transform2(3,1) = extrinsic2.at<double>(3,1);
      transform2(3,2) = extrinsic2.at<double>(3,2);
      transform2(3,3) = extrinsic2.at<double>(3,3);
      inv_transform2 = transform2.inverse(); //逆序
      
      //传入外参
      transform3(0,0) = extrinsic3.at<double>(0,0);
      transform3(0,1) = extrinsic3.at<double>(0,1);
      transform3(0,2) = extrinsic3.at<double>(0,2);
      transform3(0,3) = extrinsic3.at<double>(0,3);
      transform3(1,0) = extrinsic3.at<double>(1,0);
      transform3(1,1) = extrinsic3.at<double>(1,1);
      transform3(1,2) = extrinsic3.at<double>(1,2);
      transform3(1,3) = extrinsic3.at<double>(1,3);
      transform3(2,0) = extrinsic3.at<double>(2,0);
      transform3(2,1) = extrinsic3.at<double>(2,1);
      transform3(2,2) = extrinsic3.at<double>(2,2);
      transform3(2,3) = extrinsic3.at<double>(2,3);
      transform3(3,0) = extrinsic3.at<double>(3,0);
      transform3(3,1) = extrinsic3.at<double>(3,1);
      transform3(3,2) = extrinsic3.at<double>(3,2);
      transform3(3,3) = extrinsic3.at<double>(3,3);
      inv_transform3 = transform3.inverse(); //逆序
      //传入外参
      transform4(0,0) = extrinsic4.at<double>(0,0);
      transform4(0,1) = extrinsic4.at<double>(0,1);
      transform4(0,2) = extrinsic4.at<double>(0,2);
      transform4(0,3) = extrinsic4.at<double>(0,3);
      transform4(1,0) = extrinsic4.at<double>(1,0);
      transform4(1,1) = extrinsic4.at<double>(1,1);
      transform4(1,2) = extrinsic4.at<double>(1,2);
      transform4(1,3) = extrinsic4.at<double>(1,3);
      transform4(2,0) = extrinsic4.at<double>(2,0);
      transform4(2,1) = extrinsic4.at<double>(2,1);
      transform4(2,2) = extrinsic4.at<double>(2,2);
      transform4(2,3) = extrinsic4.at<double>(2,3);
      transform4(3,0) = extrinsic4.at<double>(3,0);
      transform4(3,1) = extrinsic4.at<double>(3,1);
      transform4(3,2) = extrinsic4.at<double>(3,2);
      transform4(3,3) = extrinsic4.at<double>(3,3);
      inv_transform4 = transform4.inverse(); //逆序
      imageSize = img_size;
      color_distance = color_dis;
      show_colored_cloud = show_cloud;
      camera_sub1  = new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_topic1,3000);
      camera_sub2  = new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_topic2,3000);
      camera_sub3  = new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_topic3,3000);
      camera_sub4  = new message_filters::Subscriber<sensor_msgs::Image>(nh, camera_topic4,3000);
      lidar_sub  = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, lidar_topic,10000);
      sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(1000), *camera_sub1, *camera_sub2,*camera_sub3,*camera_sub4,*lidar_sub);
      sync_->registerCallback(boost::bind(&RsCamFusion::callback,this, _1, _2, _3, _4,_5));
      cout<<"waite_image"<<endl;
      allocateMemory(); //初始化
      
}  
//**********************************************************************************************************
//3、成员函数
    void allocateMemory()
    {
        colored_cloud_toshow.reset(new pcl::PointCloud<PointXYZRGBI>());
        input_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        out_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        input_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
        transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        colored_cloud.reset(new pcl::PointCloud<PointXYZRGBI>());
        colored_cloud_transback.reset(new pcl::PointCloud<PointXYZRGBI>());
        lidar_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    void resetParameters(){ 
        input_cloud_ptr->clear();
        input_cloud->clear();
        out_cloud->clear();
        transformed_cloud->clear();
        colored_cloud->clear();
        colored_cloud_transback->clear();
        colored_cloud_toshow->clear();
    }
    void callback(const sensor_msgs::ImageConstPtr input_image_msg1,
                const sensor_msgs::ImageConstPtr input_image_msg2,
                const sensor_msgs::ImageConstPtr input_image_msg3,
                const sensor_msgs::ImageConstPtr input_image_msg4,
                const sensor_msgs::PointCloud2ConstPtr input_cloud_msg
                )
    {
      resetParameters();
      cv::Mat input_image1,input_image2,input_image3,input_image4;
      cv::Mat undistorted_image;
      cv_bridge::CvImagePtr cv_ptr1,cv_ptr2,cv_ptr3,cv_ptr4; 
      
      std_msgs::Header image_header1 = input_image_msg1->header;
      std_msgs::Header image_header2 = input_image_msg2->header;
      std_msgs::Header image_header3 = input_image_msg3->header;
      std_msgs::Header image_header4 = input_image_msg4->header;
      std_msgs::Header cloud_header = input_cloud_msg->header;
      
//数据获取
//图像ROS消息转化
      try
      {
        cv_ptr1 = cv_bridge::toCvCopy(input_image_msg1, sensor_msgs::image_encodings::BGR8);
        cv_ptr2 = cv_bridge::toCvCopy(input_image_msg2, sensor_msgs::image_encodings::BGR8);
        cv_ptr3 = cv_bridge::toCvCopy(input_image_msg3, sensor_msgs::image_encodings::BGR8);
        cv_ptr4 = cv_bridge::toCvCopy(input_image_msg4, sensor_msgs::image_encodings::BGR8);
      }
      catch(cv_bridge::Exception e)
      {
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
      }
      input_image1 = cv_ptr1->image;
      input_image2 = cv_ptr2->image;
      input_image3 = cv_ptr3->image;
      input_image4 = cv_ptr4->image;
//获取点云
      

      pcl::fromROSMsg(*input_cloud_msg, *input_cloud_ptr);//把input_cloud_msg放入input_cloud_ptr中
      std::vector<int> indices; 
      pcl::removeNaNFromPointCloud(*input_cloud_ptr, *input_cloud_ptr, indices);//去除无效点
      int m1=1;
      int m2=2;
      int m3=3;
      int m4=4;
      transformpoint(input_cloud_ptr,input_image1,m1);
      //transformpoint(input_cloud_ptr,input_image2,m2);
      transformpoint(input_cloud_ptr,input_image3,m3);
      //transformpoint(input_cloud_ptr,input_image4,m4);
      //发布色彩点云 和图像
      colored_cloud_showpub = nh.advertise<sensor_msgs::PointCloud2>("colored_cloud_toshow",100);
      //cout<<*colored_cloud_toshow<<endl;
      publishCloudtoShow(colored_cloud_showpub, cloud_header, colored_cloud_toshow);
      fused_image_pub1 = nh.advertise<sensor_msgs::Image>("image_to_show1",100);
      fused_image_pub2 = nh.advertise<sensor_msgs::Image>("image_to_show2",100);
      fused_image_pub3 = nh.advertise<sensor_msgs::Image>("image_to_show3",100);
      fused_image_pub4 = nh.advertise<sensor_msgs::Image>("image_to_show4",100);
      publishImage(fused_image_pub1, image_header1, image_to_show1);
      publishImage(fused_image_pub2, image_header2, image_to_show2);
      publishImage(fused_image_pub3, image_header3, image_to_show3);
      publishImage(fused_image_pub4, image_header4, image_to_show4);
      frame_count = frame_count + 1;
      }
//发布点云图像函数
    void publishImage(const ros::Publisher& image_pub, const std_msgs::Header& header, const cv::Mat image)
    {
      cv_bridge::CvImage output_image;
      output_image.header = header;
      output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      output_image.image = image;
      image_pub.publish(output_image);
    } 

    void publishCloudtoShow(const ros::Publisher& cloudtoshow_pub, const std_msgs::Header& header,
                      const pcl::PointCloud<PointXYZRGBI>::ConstPtr& cloud)
    {
      sensor_msgs::PointCloud2 output_msg;
      pcl::toROSMsg(*cloud, output_msg);
      output_msg.header = header;
      cloudtoshow_pub.publish(output_msg);
    }
    
//点云微调
    void transformpoint(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input, const cv::Mat input_image, const int m)
    {
//微调点云
        Eigen::Matrix4d transform;
        Eigen::Matrix4d inv_transform;
        if (m==1)
        {
            transform = transform1;
            inv_transform = inv_transform1;
            
        }
        else if (m==2)
        {
            transform = transform2;
            inv_transform = inv_transform2;
        }
        else if (m==3)
        {
            transform = transform3;
            inv_transform = inv_transform3;
        }
        else
        {
            transform = transform4;
            inv_transform = inv_transform4;
        }
        
        *input_cloud = *input;
        
        
        // 创建矩阵对象transform_2.matrix()，初始化为4×4单位阵
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        // 定义在x轴上的平移，2.5m
        transform_2.translation() << 0.0, 0.0, 0;	// 三个数分别对应X轴、Y轴、Z轴方向上的平移
        // 定义旋转矩阵，绕z轴M_PI/8
        transform_2.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));	//同理，UnitX(),绕X轴；UnitY(),绕Y轴.
        // 打印平移、旋转矩阵
        //std::cout << "\n方式2: 使用Affine3f\n";
        //std::cout << transform_2.matrix() << std::endl;	//注意：不是transform_2

        // 执行转换
        // transform_1 或者 transform_2 都可以实现相同的转换
        //cout<<*input_cloud<<endl;
        std::cout << transform_2.matrix() << std::endl;
        pcl::transformPointCloud(*input_cloud, *out_cloud, transform_2);	//注意：不是transform_2.matrix() 

        if (input_cloud->size() == 0)
        {
        WARN << "input cloud is empty, please check it out!" << REND;
        }

        //transform lidar points from lidar coordinate to camera coordiante
        //点云旋转变化放入transformed_cloud中
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);//new开辟了一个新的空间在局部函数进行完后仍能调用
        pcl::transformPointCloud (*out_cloud, *transformed_cloud, transform);        //lidar coordinate(forward x+, left y+, up z+)
                                                                                         //camera coordiante(right x+, down y+, forward z+) (3D-3D)  
                                                                                        //using the extrinsic matrix between this two coordinate system
                                  
        std::vector<cv::Point3d> lidar_points;
        std::vector<cv::Scalar> dis_color;
        std::vector<float> intensity;
         
        
        //4、只保留前面的点云，只投影到前面相机
        //reserve the points in front of the camera(z>0)
        for(int i=0;i<=transformed_cloud->points.size();i++)
        {
          if(transformed_cloud->points[i].z>0)
          {
            //将z正点云保存到lidar_points
            lidar_points.push_back(cv::Point3d(transformed_cloud->points[i].x, transformed_cloud->points[i].y, transformed_cloud->points[i].z));
            int color_order = int(transformed_cloud->points[i].z / color_distance);
            if(color_order > 20)
            {
              color_order = 20;
            }
            dis_color.push_back(cv::Scalar(color[color_order][2], color[color_order][1], color[color_order][0]));
            intensity.push_back(transformed_cloud->points[i].intensity);
          }
        }
//点云投影
        if (m==1)
        {
            cv::projectPoints(lidar_points, rMat, tVec, intrinsic1, distcoeff1, imagePoints); 
            
        }
        else if (m==2)
        {
            cv::projectPoints(lidar_points, rMat, tVec, intrinsic2, distcoeff2, imagePoints); 
        }
        else if (m==3)
        {
            cv::projectPoints(lidar_points, rMat, tVec, intrinsic3, distcoeff3, imagePoints); 
        }
        else
        {
            cv::projectPoints(lidar_points, rMat, tVec, intrinsic4, distcoeff4, imagePoints); 
        }

        image_to_show = input_image.clone();
        for(int i=0;i<imagePoints.size();i++)
        {
            if(imagePoints[i].x>=0 && imagePoints[i].x<640 && imagePoints[i].y>=0 && imagePoints[i].y<480)
            {
              cv::circle(image_to_show, imagePoints[i], 1, dis_color[i], 2, 8, 0);//画圆
            if (m==1)
            {
                image_to_show1 = image_to_show;
            }
            else if (m==2)
            {
                image_to_show2 = image_to_show;
            }
            else if (m==3)
            {
                image_to_show3 = image_to_show;
            }
            else
            {
            
                image_to_show4 = image_to_show;
            }
              
              

              PointXYZRGBI point;                                                             //reserve the lidar points in the range of image 
              point.x = lidar_points[i].x;                                                        //use 3D lidar points and RGB value of the corresponding pixels  
              point.y = lidar_points[i].y;                                                        //to create colored point clouds
              point.z = lidar_points[i].z;
              point.intensity = intensity[i];
              point.r = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[2];
              point.g = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[1];
              point.b = input_image.at<cv::Vec3b>(imagePoints[i].y, imagePoints[i].x)[0];
              colored_cloud->points.push_back(point);  
              
            }
        }
        //transform colored points from camera coordinate to lidar coordinate
        pcl::transformPointCloud (*colored_cloud, *colored_cloud_transback, inv_transform); //再将点云转化到三维     
        //cout<<*colored_cloud_transback<<endl;
        if(show_colored_cloud)
        {  
            
            
            for(int i=0;i<colored_cloud_transback->points.size();i++)
            {
                
                    PointXYZRGBI point;                                                             
                    point.x = colored_cloud_transback->points[i].x;                                                        
                    point.y = colored_cloud_transback->points[i].y;                                                        
                    point.z = colored_cloud_transback->points[i].z;
                    point.intensity = colored_cloud_transback->points[i].intensity;
                    colored_cloud_toshow->points.push_back (point);
                
                
                /*
                if (int(colored_cloud_transback->points[i].r) == 128 || int(colored_cloud_transback->points[i].r) == 152
                  ||int(colored_cloud_transback->points[i].r) == 107 || int(colored_cloud_transback->points[i].r) == 244
                  ||int(colored_cloud_transback->points[i].r) == 70 )
                {
                    PointXYZRGBI point;                                                             
                    point.x = colored_cloud_transback->points[i].x;                                                        
                    point.y = colored_cloud_transback->points[i].y;                                                        
                    point.z = colored_cloud_transback->points[i].z;
                    point.r = colored_cloud_transback->points[i].r;
                    point.g = colored_cloud_transback->points[i].g;
                    point.b = colored_cloud_transback->points[i].b;
                    point.intensity = colored_cloud_transback->points[i].intensity;
                    colored_cloud_toshow->points.push_back (point);
                    //cout<<int(point.r)<<","<<int(point.g)<<","<<int(point.b)<<endl;
                }
                */
                
                
           }
           
           //cout<<*colored_cloud_toshow<<endl;  
     } 
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
  ros::init(argc, argv, "add_color_node");
  RsCamFusion RF;
  ros::spin();
  return 0;
}


  
  
  
  
  
  
  
  
  
  
  

