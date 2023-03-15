// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "utility.h"


class ImageProjection{
private:
    //1、定义接受发布者
    ros::NodeHandle nh;
    
    ros::Subscriber subLaserCloud;
    
    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;
    //2、定义点云类型
    pcl::PointCloud<PointType>::Ptr laserCloudIn;//雷达直接传出的点云
    pcl::PointCloud<PointType>::Ptr fullCloud;//投影后的点云
    pcl::PointCloud<PointType>::Ptr fullInfoCloud;//整体的点云
    //发布后的点云
    pcl::PointCloud<PointType>::Ptr groundCloud;//地面点云
    pcl::PointCloud<PointType>::Ptr segmentedCloud;//分割后的部分
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;//分割后的部分的几何信息
    pcl::PointCloud<PointType>::Ptr outlierCloud;//
    PointType nanPoint;
    //3、定义图像
    cv::Mat rangeMat;  //矩阵图像
    cv::Mat labelMat;
    cv::Mat groundMat;
    //4、其他变量
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info segMsg;
    std_msgs::Header cloudHeader;

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;
//×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
public:
    ImageProjection():  //构造函数 创建类时自动执行里面的函数
        nh("~"){
        // 1、订阅来自velodyne雷达驱动的topic ("/velodyne_points") /kitti/velo/pointcloud
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1, &ImageProjection::cloudHandler, this);
        // 2、发布地面点云 和 分割好的点云
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;
        // 3、申请内存
        allocateMemory();
        // 4、参数初始化
        resetParameters();
    }

	// 初始化各类参数以及分配内存
    void allocateMemory(){

        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

		// labelComponents函数中用到了这个矩阵
		// 该矩阵用于求某个点的上下左右4个邻接点
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

	// 初始化/重置各类参数内容
    void resetParameters(){
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }
//详细操作××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    ~ImageProjection(){}
	

    // 点云处理回调函数   传入参数是消息
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        copyPointCloud(laserCloudMsg);  //1、将ROS消息转化成pcl点云
        findStartEndAngle();            //2、扫描开始和结束的角度
        projectPointCloud();            //3、距离图像投影
        groundRemoval();                //4、标记地面点云
        cloudSegmentation();            //5、点云分割
        publishCloud();                 //6、发布分割后的点云
        resetParameters();              //7、重置所有参数
    }
    
    //1、将ROS消息转化成pcl点云 
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // × 读取ROS点云转换为PCL点云
        cloudHeader = laserCloudMsg->header;
        // cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        // ×× 运用pcl库中的fromROSMsg将laserCloudMsg ROS消息 转化在laserCloudIn 中pcl点云
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
        // ××× 运用pcl库中的removeNaNFromPointCloud去除点云中的Nan points
        std::vector<int> indices; //vector是一个能够存放任意类型的动态数组，能够增加和压缩数据。 创建一个空对象
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
        // ×××× 这里对velodyne的lidar做了区分处理，elodyne的lidar对点云属于16线中的哪一线做了标记
        // 如果点云有"ring"通过，则保存为laserCloudInRing
        //××××× 现在点云分别保存为"laserCloudIn"和"laserCloudInRing"中
    }
    
    
    //2、扫描开始和结束的角度   雷达旋转一周发一次数据，这里是看起始角度和末角度
    void findStartEndAngle(){
        // 雷达坐标系：右->X,前->Y,上->Z
        // 雷达内部旋转扫描方向：Z轴俯视下来，顺时针方向（Z轴右手定则反向）

        // atan2(y,x)函数的返回值范围(-PI,PI],表示与复数x+yi的幅角
        // segMsg.startOrientation范围为(-PI,PI]
        // segMsg.endOrientation范围为(PI,3PI]
        // 因为内部雷达旋转方向原因，所以atan2(..)函数前面需要加一个负号
        // × 在laserCloudIn保存的点云中，根据point[0]的xy坐标找开始角度
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        // ×× 在laserCloudIn保存的点云中，根据point[size]最后的点云的xy坐标找结束角度
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI; //M_PI是c++语言中标准库<math.h>定义的宏，值为3.14
		
        // 开始和结束的角度差一般是多少？
		// 一个velodyne 雷达数据包转过的角度多大？
        // 雷达一般包含的是一圈的数据，所以角度差一般是2*PI，一个数据包转过360度
		// segMsg.endOrientation - segMsg.startOrientation范围为(0,4PI)
        // 如果角度差大于3Pi或小于Pi，说明角度差有问题，进行调整。

        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
		// 开始角度与结束角度差
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
    }
    
    //3、距离图像投影
    void projectPointCloud(){
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; //size_t是表示长度（尺寸）的类型
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();  //点云的尺寸

        for (size_t i = 0; i < cloudSize; ++i){
            //+ 获取每个点云的坐标
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.r = laserCloudIn->points[i].r;
            thisPoint.g = laserCloudIn->points[i].g;
            thisPoint.b = laserCloudIn->points[i].b;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            //++ 计算竖直方向上的角度（雷达的第几线）
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
			
            //+++ rowIdn计算出该点激光雷达是竖直方向上第几线的
			// 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y; //ang_bottom在.h文件中定义
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;
            // 下方角度atan2(..)交换了x和y的位置，计算的是与y轴正方向的夹角大小(关于y=x做对称变换)
            //+++ 点云正前方的夹角大小    右x前y
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            //++++ 距离图像的横坐标  round函数进行四舍五入取整
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            // 结果大于一周就是一周多  其他继续
            if (columnIdn >= Horizon_SCAN) columnIdn -= Horizon_SCAN;   //ang_res_x  Horizon_SCAN在.h文件中定义
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;

            //+++++ 极坐标的P,也就是点和激光雷达的距离
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            //保存距离到矩阵rangeMat
            rangeMat.at<float>(rowIdn, columnIdn) = range;

			// 强度实际上为了保持纵坐标和横坐标
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            //++++++ 列：COL 行：row
            index = columnIdn  + rowIdn * Horizon_SCAN; //计算每个点的索引
            fullCloud->points[index] = thisPoint;  //点云保存到数组
            fullInfoCloud->points[index].intensity = range; //距离保存到数组
        }
        
    }

    //4、标记地面点云
    void groundRemoval(){
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // +、遍历纵ID小于7的点
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            // groundScanInd 是在 utility.h 文件中声明的线数，groundScanInd=7
            for (size_t i = 0; i < groundScanInd; ++i){
        // ++、 取临近两个点的ID
                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                // 初始化的时候用nanPoint.intensity = -1 填充
                // 都是-1 证明是空点nanPoint
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
        // +++、通过两点之间的xyz的值来计算俯仰角 
				// 由上下两线之间点的XYZ位置得到两线之间的俯仰角
				// 如果俯仰角在10度以内，则判定(i,j)为地面点,groundMat[i][j]=1
				// 否则，则不是地面点，进行后续操作
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

                //俯仰角
                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
         // ++++、如果小于10度，两个点都是地面点，保存到groundMat的(i,j)位置，数值为1
                //1代表地面点
                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        //+++++、将地面点和无效点保存在labelMat中
		// 找到所有点中的地面点或者距离为FLT_MAX(无效值为空)的点，并将他们标记为-1  应该是无效值空值在计算距离时结果是最大正值FLT_MAX   后面要移除地面点和无效点
        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;  //将这些点保存在labelMat里面
                }
            }
        }
        // ++++++、发布数据
		// getNumSubscribers函数如果有节点订阅groundCloud，那么就需要把地面点发布出来
		// 具体实现过程：把点放到groundCloud队列中去
	    // 根据是否有节点订阅所需话题，决定是否发布对应话题
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]); 
    //将i，j表示地面点云对应的fullcloud即点云原始数据发出，push与push_back是STL中常见的方法，都是向数据结构中添加元素
                }
            }
        }
    }
    //5、点云分割
    void cloudSegmentation(){
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
				// 之前将地面点和无效点保存到labelmat中值为-1,在剩下为0的点去找特征点
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);  //将特征明显的点传入labelComponents函数
    // 到此已经标记好所有点云  一样的序号表示一类
        int sizeOfSegCloud = 0;
        for (size_t i = 0; i < N_SCAN; ++i) {
			
			// segMsg.startRingIndex[i]
			// segMsg.endRingIndex[i]
			// 表示第i线的点云起始序列和终止序列
			// 以开始线后的第6线为开始，以结束线前的第6线为结束
            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
				// 找到可用的特征点或者地面点(不选择labelMat[i][j]=0的点)
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
					// labelMat数值为999999表示这个点是因为聚类数量不够30而被舍弃的点
					// 需要舍弃的点直接continue跳过本次循环，
					// 当列数为5的倍数，并且行数较大，可以认为非地面点的，将它保存进异常点云(界外点云)中
	                //× 如果label为999999则跳过
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);//将异常点云保存到outlierCloud中
                            continue;
                        }else{
                            continue;
                        }
                    }
                    
					// ×× 如果为地，跳过index不是5的倍数的点
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
					// 上面多个if语句已经去掉了不符合条件的点，这部分直接进行信息的拷贝和保存操作
					// 保存完毕后sizeOfSegCloud递增
                    segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    ++sizeOfSegCloud;
                }
            }

            // 以结束线前的第5线为结束
            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;   //前5帧和后5帧省去
        }

		// 如果有节点订阅SegmentedCloudPure,
		// 那么把点云数据保存到segmentedCloudPure中去
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
					// 需要选择不是地面点(labelMat[i][j]!=-1)和没被舍弃的点
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }


    //BFS递归找特征比较明显的点
    void labelComponents(int row, int col){
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        // 标准的BFS
        // BFS的作用是以(row，col)为中心向外面扩散，
    // × 判断(row,col)是否是这个平面中一点
        while(queueSize > 0){
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
			// labelCount的初始值为1，后面会递增  表示类别序号1 2 3 4 5 6
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

			// neighbor=[[-1,0];[0,1];[0,-1];[1,0]]
	// ×× 遍历点[fromIndX,fromIndY]边上的四个邻点
			//之前定义了neighborIterator有四步
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){ 

                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;
    // ××× 判断上下左右的点是否有效  x是纵坐标 y是横坐标左右联通   环状图片
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;

                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
				// 如果点[thisIndX,thisIndY]已经标记过
				// labelMat中，-1代表无效点，0代表未进行标记过，其余为其他的标记
				// 如果当前的邻点已经标记过，则跳过该点。
	// ×××× 聚类
				// 如果labelMat已经标记为正整数，则已经聚类完成，不需要再次对该点聚类
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),  // fromInd是中心点  thisInd是上下左右点
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
    //××××× 计算纵向角度和横向角度
				// alpha代表角度分辨率，
				// X方向上角度分辨率是segmentAlphaX(rad)  在.h文件定义
				// Y方向上角度分辨率是segmentAlphaY(rad)
                if ((*iter).first == 0)    //左右位置
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

				// 通过下面的公式计算这两点之间是否有平面特征
				// atan2(y,x)的值越大，d1，d2之间的差距越小,越平坦
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){
					// segmentTheta=1.0472<==>60度
					// 如果算出角度大于60度，则假设这是个平面
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount; //将旁边的点与中心点化为一类
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;  //allPushedInd为一个类别
                    ++allPushedIndSize;
                }
            }
        }


        bool feasibleSegment = false;
        
		// 如果聚类超过30个点，直接标记为一个可用聚类，labelCount需要递增
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
			// 如果聚类点数小于30大于等于5，统计竖直方向上的聚类点数
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;

			// 竖直方向上超过3个也将它标记为有效聚类
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }

        if (feasibleSegment == true){   //feasibleSegment == true代表聚完一类
            ++labelCount;
        }else{
            for (size_t i = 0; i < allPushedIndSize; ++i){
				// 标记为999999的是需要舍弃的聚类的点，因为他们的数量小于30个
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    // 发布各类点云内容
    void publishCloud(){
    	// 发布cloud_msgs::cloud_info消息
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);

        sensor_msgs::PointCloud2 laserCloudTemp;

		// pubOutlierCloud发布界外点云
        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);

		// pubSegmentedCloud发布分块点云
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);

        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }

        //if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundCloud.publish(laserCloudTemp);
        //}

        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }

        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};


//一、点云分割
int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;  //建立对象主要工作流

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
