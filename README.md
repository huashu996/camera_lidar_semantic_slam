# camera-lidar-slam
We propose a semantic segmentation odometry and mapping method based on LIDAR and camera data vision fusion for real-time motion states estimation and high-level
understanding of the surrounding environment.
# RUN
## 跑自己数据集
- 1.启动deeplabv3 ROS节点
```
cd DeepLabV3Plus-Pytorch
conda activate deeplabv3
python predict_ros.py
```
- 2.启动照片投影点云节点
```
catkin_make or catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch all_add_color all_add_color.launch 

```
- 3.启动lego_loam
```
source devel/setup.bash
roslaunch lego_loam run.launch 

```
- 4.启动rosbag
```
rosbag play jixie.bag --clock

```
## 跑kitti数据集
- 1.启动deeplabv3 ROS节点 注意需要改话题
```
cd DeepLabV3Plus-Pytorch
conda activate deeplabv3
python predict_ros.py
```
- 2.启动照片投影点云节点，只有前视投影
```
catkin_make or catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
roslaunch kittivelo_cam kittivelo_cam.launch 

```
- 3.启动lego_loam 注意改话题
```
source devel/setup.bash
roslaunch lego_loam run.launch 

```
- 4.启动kitti的bag
```
rosbag play kitti.bag --clock
```
# 注意事项
## ubuntu20.04跑lego_loam
Q:编译不报错，能够正常运行，但不显示地图和轨迹，只看到坐标系移动
A:在mapOptmization.cpp文件中将以下代码中的/camera_init更改为camera_init，去除/
## 激光雷达的线数修改
在LeGO-LOAM/includeutility.h文件中
## 如果用自己的相机，需要修改参数
color_point/src/all_add_color/config/calib_result.yaml

