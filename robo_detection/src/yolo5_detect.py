"""Run inference with a YOLOv5 model on images 

Usage:
    
"""

import numpy as np
import torch
import cv2
import torch.backends.cudnn as cudnn
import os, sys
import yaml


currentdir = os.path.dirname( os.path.realpath(__file__) )
sys.path.append(currentdir+'/yolov5/')


from yolov5.models.experimental import attempt_load
from yolov5.utils.general import check_img_size, non_max_suppression
from yolov5.utils.torch_utils import select_device
from yolov5.utils.augmentations import letterbox


class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))


class yolo5_detector():
    def __init__(self):


        # self.parse_variables(currentdir + "/detection_config.ini") 
        config_path =  os.path.join(currentdir, "../config" + "/detection_config.yaml")
        if os.path.exists(config_path):
            self.load_param(config_path) 
        else:
            raise FileNotFoundError("No camera yaml file is found!")
        
        
        self.colors = Colors()
        
        self.max_det=1000            # maximum detections per image
        self.device='cuda:0'         # cuda device i.e. 0 or 0123 or cpu
        self.classes=None            # filter by class: --class 0 or --class 0 2 3
        self.agnostic_nms=False      # class-agnostic NMS
        self.augment=False           # augmented inference
        self.half=False              # use FP16 half-precision inference


        device = select_device(self.device)
        # self.half &= device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        print("----------loading model----------")
        self.model = attempt_load( self.model_path, map_location=device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # self.model stride
        imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size
        # names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
        print("-----------Done-----------")

        if self.half:
            self.model.half()  # to FP16

        cudnn.benchmark = True




    def predict(self, img):
        """ objects prediction in img

        Args:
            img (cv2): cv2 image, BGR ordered 

        Returns:
            [list of tensor]: each of the tensor contains:
            [0-3]: bbox, 0,1 left top; 2,3 right bottom
            [4]: probability
            [5]: name
        """
        img_dim_x = img.shape[0]
        img_dim_y = img.shape[1]
        self.new_img_dim = (int(img_dim_y * self.imgsz/img_dim_y), int(img_dim_x * self.imgsz/img_dim_y))
        # print("new image dim ", self.new_img_dim)
        
        img = letterbox(img, self.imgsz, stride=self.stride)[0]
        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        # print(img.shape)
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        # pred = self.model(img,augment=self.augment,visualize=increment_path(save_dir / Path(path).stem, mkdir=True) if self.visualize else False)[0]
        pred = self.model(img,augment=self.augment, visualize=False)[0]

        # Apply NMS 
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det) 
        
        return pred


    def box_label(self, pred, image):

        image = cv2.resize(image,self.new_img_dim)
        # loop through the detection result 
        for i in range(pred.shape[0]):

            bbox = pred[i,0:4]
            start_point = ( int( bbox[0] ) , int( bbox[1]  )  )
            end_point =   ( int( bbox[2] ) , int( bbox[3] )   )
            
            class_id = int(pred[i,5]) 
            # print(class_id)
            # class_name = self.names[class_id]
            class_name = "to do here"
            color = self.colors(class_id, True) # assign color for drawing bbox

            text_out = class_name + ': ' + "{:.2f}".format(round(pred[i, 4].item(), 2))
            image = cv2.rectangle( image, start_point, end_point, color, self.bbox_line_thickness)
            image = cv2.putText(image, text_out, (start_point[0], start_point[1] - 5), 0, 0.5, color, thickness=self.bbox_line_thickness)
            
        if self.visualize == True:
            self.image_plot(image)
        return image

    def load_param(self, yaml_path):
        with open(yaml_path, "r") as f:
            self.params = yaml.load(f, Loader=yaml.CLoader)
        if self.params is not None:
            self.parse_variables()

    def parse_variables(self):

        self.model_path= currentdir + self.params['MODEL']['model_path']  # model.pt path(s)
        self.imgsz= self.params['MODEL']['infe_image_size']  # inference size (pixels) , one number
        self.names = self.params['MODEL']['names']
        self.conf_thres = self.params['MODEL']['conf_thres'] # confidence threshold
        self.iou_thres=self.params['MODEL']['iou_thres']       # NMS IOU threshold
        self.bbox_line_thickness = self.params['VISUAL']['line_thickness']
        self.visualize = self.params['VISUAL']['visualize']
        print(self.visualize)


    # def parse_variables(self, config_file):
    #     parser = configparser.ConfigParser()
    #     parser.read(config_file)

    #     self.model_path= currentdir + parser['MODEL']['model_path']  # model.pt path(s)
    #     self.imgsz=parser.getint('MODEL','infe_image_size')  # inference size (pixels) , one number
    #     self.names = parser.get('MODEL','names')
    #     self.names = json.loads(self.names)
    #     self.conf_thres = parser.getfloat('MODEL','conf_thres') # confidence threshold
    #     self.iou_thres=parser.getfloat('MODEL','iou_thres')       # NMS IOU threshold
    #     self.bbox_line_thickness = parser.getint('VISUAL','line_thickness')
    #     self.visualize = parser.getboolean('VISUAL','visualize')
    #     print(self.visualize)

    
    def get_class_name(self,class_id):
        class_id = int(class_id) 
        class_name = self.names[class_id]
        
        return class_name
    
    def get_class_id(self, class_name):
        class_id = self.names.index(class_name)   
        return class_id
    
    @staticmethod
    def image_plot(image, wait = 1):
        cv2.imshow('frame', image)
        cv2.waitKey(wait)
        


if __name__== "__main__":
    print("test")
    image = cv2.imread('/home/robo-dell/Downloads/construction-safety.jpg',cv2.IMREAD_COLOR)


    yolo = yolo5_detector()
    pred = yolo.predict(image)[0]
    print(pred)
    print(len(pred))
    image_out = yolo.box_label(pred, image)
    # cv2.imshow('frame', image_out)
    # cv2.waitKey(0)


