import roslib
import rospy
from std_msgs.msg import Header
from std_msgs.msg import String
from sensor_msgs.msg import Image

from torch.utils.data import dataset
from tqdm import tqdm
import network
import utils
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "0"
import random
import argparse
import numpy as np

from torch.utils import data
from datasets import VOCSegmentation, Cityscapes, cityscapes
from torchvision import transforms as T
from metrics import StreamSegMetrics

import torch
import torch.nn as nn

from PIL import Image as IM
import matplotlib
import matplotlib.pyplot as plt

def get_argparser():
    parser = argparse.ArgumentParser()

    # Deeplab Options
    available_models = sorted(name for name in network.modeling.__dict__ if name.islower() and \
                              not (name.startswith("__") or name.startswith('_')) and callable(
                              network.modeling.__dict__[name])
                              )

    parser.add_argument("--separable_conv", action='store_true', default=False,
                        help="apply separable conv to decoder and aspp")
    parser.add_argument("--output_stride", type=int, default=16, choices=[8, 16])

    # Train Options

    parser.add_argument("--crop_val", action='store_true', default=False,
                        help='crop validation (default: False)')
    parser.add_argument("--val_batch_size", type=int, default=4,
                        help='batch size for validation (default: 4)')
    parser.add_argument("--crop_size", type=int, default=513)

    
    parser.add_argument("--gpu_id", type=str, default='0',
                        help="GPU ID")
    return parser

class SubscribeAndPublish:
    def __init__(self):
        self.all_obstacle_str=''
        self.sub1_name="/cam_rgb3/usb_cam/image_raw"
        print("waite")
        self.sub1= rospy.Subscriber(self.sub1_name,Image,self.callback_rgb1)
        
        self.pub1_name="forward"
        self.pub1= rospy.Publisher(self.pub1_name, Image,queue_size=10)
        self.model=model
        self.img_rgb=[]
        self.colorized_preds=[]
        self.colorized_preds2=[]
        self.timestr=[]
    def callback_rgb1(self,data):
        print('callback')
        with torch.no_grad():
            self.timestr = data.header.stamp
            print(self.timestr)
            model = self.model.eval()
            img_rgb = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            print(img_rgb.shape)
            print(type(img_rgb))
            #img_rgb=img_rgb[:,:,::-1]#的作用就是实现RGB到BGR通道的转换 （若图片一开始就是BGR的，就是实现从BGR到RGB的转换）     
            img_rgb = transform(img_rgb).unsqueeze(0) # To tensor of NCHW
            print(img_rgb.shape) #([1, 3, 480, 640])
            img_rgb = img_rgb.to(device)
            pred = model(img_rgb).max(1)[1].cpu().numpy()[0] # HW
            colorized_preds = decode_fn(pred).astype('uint8')
            print(colorized_preds)
            
            self.publish_image(self.pub1,colorized_preds,'base_link')
    def publish_image(self,pub, data, frame_id='base_link'):
        #shape是numpy的使用方法，PIL图片不能发送
        assert len(data.shape) == 3, 'len(data.shape) must be equal to 3.'
        time = self.timestr
        print(time)
        header = Header(stamp=time)
        header.frame_id = frame_id
        msg = Image()
        msg.height = data.shape[0]
        msg.width = data.shape[1]
        msg.encoding = 'rgb8'
        msg.data = np.array(data).tostring()
        msg.header = header
        msg.step = msg.width * 1 * 3
        pub.publish(msg)


def mian(opts,model,device):          
    rospy.init_node('sem_image', anonymous=True)
    #####################
    t=SubscribeAndPublish()
    #####################
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        rate.sleep()
if __name__ == '__main__':
    opts = get_argparser().parse_args()
    weights = '/home/cxl/sem_slam/src/DeepLabV3Plus-Pytorch/weights/best_deeplabv3plus_mobilenet_cityscapes_os16.pth'
    dataset = 'cityscapes'
    model_choice = 'deeplabv3plus_mobilenet'
    if dataset.lower() == 'cityscapes':
        opts.num_classes = 19
        decode_fn = Cityscapes.decode_target
    
    #device = torch.device('cuda:0')
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print("Device: %s" % device)
    #加载模型
    model = network.modeling.__dict__[model_choice](num_classes=opts.num_classes, output_stride=opts.output_stride)
    if opts.separable_conv and 'plus' in model_choice:
        network.convert_to_separable_conv(model.classifier)
    utils.set_bn_momentum(model.backbone, momentum=0.01)
    
    if weights is not None and os.path.isfile(weights):
        # https://github.com/VainF/DeepLabV3Plus-Pytorch/issues/8#issuecomment-605601402, @PytaichukBohdan
        checkpoint = torch.load(weights, map_location=torch.device('cpu'))
        model.load_state_dict(checkpoint["model_state"])
        model = nn.DataParallel(model)
        model.to(device)
        print("Resume model from %s" % weights)
        del checkpoint
    else:
        print("[!] Retrain")
        model = nn.DataParallel(model)
        model.to(device)
    if opts.crop_val:
        transform = T.Compose([
                T.Resize(opts.crop_size),
                T.CenterCrop(opts.crop_size),
                T.ToTensor(),
                T.Normalize(mean=[0.485, 0.456, 0.406],
                                std=[0.229, 0.224, 0.225]),
            ])
    else:
        transform = T.Compose([
                T.ToTensor(),
                T.Normalize(mean=[0.485, 0.456, 0.406],
                                std=[0.229, 0.224, 0.225]),
            ])
    mian(opts,model,device)
    

    
    
    
    
    
