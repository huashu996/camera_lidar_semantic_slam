#include "utility.h"

class TransformFusion{
private:
    ros::NodeHandle nh;
    ros::Publisher sendpoint;
    ros::Subscriber  getpoint;
    pcl::PointCloud<PointType>::Ptr color_point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered;
    pcl::PointCloud<PointType>::Ptr cloud_filtered2;
public:
    TransformFusion():
    nh("~"){
            sendpoint= nh.advertise<sensor_msgs::PointCloud2>("/less_point", 2);
            getpoint= nh.subscribe<sensor_msgs::PointCloud2>("/colored_cloud_toshow", 2, &TransformFusion::less, this);
            allocateMemory();
    }
    void allocateMemory(){
        color_point.reset(new pcl::PointCloud<PointType>());
        cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud_filtered2.reset(new pcl::PointCloud<PointType>());
        }
    void less(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        cloud_filtered->clear();
        pcl::fromROSMsg(*msg, *color_point);
        pcl::VoxelGrid<PointType> sor;//创建滤波器对象
        sor.setInputCloud (color_point);//设置输入点云
        sor.setLeafSize (0.2, 0.2, 0.2);//体素大小设置为10*10*10cm
        sor.filter (*cloud_filtered2);//执行滤波，保存过滤结果在cloud_filtered

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i=0;i<cloud_filtered2->points.size();i++)
        {
            pcl::PointXYZRGB point;                                                             
            point.x = cloud_filtered2->points[i].x;                                                        
            point.y = cloud_filtered2->points[i].y;                                                        
            point.z = cloud_filtered2->points[i].z;
            point.r = cloud_filtered2->points[i].r;
            point.g = cloud_filtered2->points[i].g;
            point.b = cloud_filtered2->points[i].b;
            cloud_filtered->points.push_back (point); 
            int r = cloud_filtered2->points[i].r;
            int g = cloud_filtered2->points[i].g;
            int b = cloud_filtered2->points[i].b;
            cout<<r<<","<<g<<","<<b<<endl;
          }
        cout<<cloud_filtered->points.size()<<endl;
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*cloud_filtered, cloudMsgTemp);
        cloudMsgTemp.header.frame_id = "/velodyne";
        sendpoint.publish(cloudMsgTemp);
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lego_loam");
    
    TransformFusion TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

    ros::spin();

    return 0;
}
