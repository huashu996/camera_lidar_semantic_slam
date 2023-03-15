## 环境配置
详细教程见博客
https://blog.csdn.net/HUASHUDEYANJING/article/details/123959970
- 新建环境
```
conda create -n deeplabv3 python=3.8
conda activate deeplabv3
```
- 安装pytorch
```
conda install pytorch torchvision cudatoolkit=11.3 -c pytorch
//11.3为电脑cuda的版本，其他版本也可以
```
- 安装deeplabv3
```
git clone https://github.com/huashu996/Deeplabv3plus_ros_pytorch
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install visdom
```
- 下载数据集和模型
```
数据集：链接: https://pan.baidu.com/s/1eiPyD6Esjihiph9yCTYv0Q 提取码: qrcl
权重：链接: https://pan.baidu.com/s/1-CE9WUVkyhg64YD6IDwL9g 提取码: t4wt
下载到DeepLabV3Plus-Pytorch/datasets/data目录下并解压
解压命令
tar xvf VOCtrainval_06-Nov-2007.tar
tar xvf VOCtest_06-Nov-2007.tar
tar xvf VOCtrainval_11-May-2012.tar
将训练模型放在weight文件夹中
```
## 测试运行

- 测试
```
#单张图片
python predict.py --input datasets/data/test/1.jpg --dataset voc --model deeplabv3plus_mobilenet --ckpt weights/best_deeplabv3plus_mobilenet_voc_os16.pth --save_val_results_to test_results
#文件夹图片
python predict.py --input datasets/data/test --dataset voc --model deeplabv3plus_mobilenet --ckpt weights/best_deeplabv3plus_mobilenet_voc_os16.pth --save_val_results_to test_results
#cityscapes
python predict.py --input datasets/data/JPEGImages --dataset cityscapes --model deeplabv3plus_mobilenet --ckpt weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth --save_val_results_to test_results
```
- 训练
```
python main.py --model deeplabv3plus_mobilenet --dataset cityscapes --enable_vis --vis_port 8097 --gpu_id 0  --lr 0.1  --crop_size 321 --batch_size 2 --output_stride 16 --data_root ./datasets/data/cityscapes --ckpt weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth
batch_size根据电脑性能做调整
num_classes 为种类个数+1
--ckpt 为预训练权重文件
```
- 验证
```
python main.py --model deeplabv3plus_mobilenet --dataset cityscapes --enable_vis --vis_port 8097 --gpu_id 0 --year 2012_aug --crop_val --lr 0.01 --crop_size 513 --batch_size 16 --output_stride 16 --ckpt weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth --test_only --save_val_results

```
- ROS节点
在predict_ros.py中修改weight
weights = './weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth'
```
roscore
python predict_ros.py --ckpt weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth

```








