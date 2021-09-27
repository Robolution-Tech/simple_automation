#!/usr/bin/env python


from yolo5_detect import yolo5_detector
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_from_euler

import rospy
import configparser
import os, sys
import numpy as np
import cv2
import json
import tf

currentdir = os.path.dirname( os.path.realpath(__file__) )

ESTIMATION_TYPE = {"NAIVE": "naive",
                   "LIDAR_SEG": "lidar_seg"}


class object_pose_estimation():

    def __init__(self):
        rospy.init_node("object_pose_estimation_node", anonymous=True)
        self.br = tf.TransformBroadcaster()
        self.parse_variables(currentdir + "/detection_config.ini") 
        self.img = None
        
        
        self.detector = yolo5_detector()
    
        self.cam_subscriber = rospy.Subscriber(self.cam_topic_name, CompressedImage, self.cam_callback,  queue_size = 1)
        # self.lidar_subscriber = rospy.Subscriber(self.lidar_topic_name, PointCloud2, self.lidar_callback)

    def naive_estimation(self,pred,class_name):
        """naive method to estimate object position"
        
        distance = obj_real_size(in m) /pixel_size(in m) * focal_length
        pixel_size = obj_pixel_y/img_pixel_y (reshaped) * img_pixel_y(origin) * n_pixel 
        Args:
            image ([type]): [description]
        """

        obj_num = len(pred)
        obj_pose_list = []
        
        #get image size

        # image = cv2.resize( self.img, self.detector.new_img_dim )        
        
        img_x = self.detector.new_img_dim[0]
        img_y = self.detector.new_img_dim[1]
        
        #TODO: determin which obj is the right one to use
        #compute pose for each class_name obj
        for i in range(obj_num):        
            
            #compute bbox size
            bbox = pred[i,0:4]
            prob = pred[i, 4].item()
            class_id = self.detector.get_class_id(class_name)
            class_id_pred = pred[i,5]
            if class_id == class_id_pred:
                start_point = ( int( bbox[0] ) , int( bbox[1]  )  )
                end_point =   ( int( bbox[2] ) , int( bbox[3] )   )
                
                len_x = end_point[0] - start_point[0]
                len_y = end_point[1] - start_point[1]
                
                #compute relative pose x,y
                pose_x = (img_x/ len_x + img_y/len_y) * self.naive_scale  * self.naive_obj_x
                print("pose_x to obj", i, "is: ", pose_x)
                #pose_y  is equal to (bbox_mid - img_mid) / bbox_x*obj_x
                pose_y = ((end_point[0]-start_point[0]) - img_x/2 ) / len_x * self.naive_obj_x
                print("pose_y to obj", i, "is: ", pose_y)
                
                self.send_obj_tf((pose_x,pose_y), "base_link", "obj1")
                
        #tf transfer to base link        
                
    
    
    def parse_variables(self, config_file):
        parser = configparser.ConfigParser()
        parser.read(config_file)

        
        #sensor config
        self.cam_topic_name= parser['SENSOR']['cam_topic_name']
        self.lidar_topic_name= parser['SENSOR']['lidar_topic_name']
        
        
        #estimation confg 
        self.pose_estimate_method = parser['ESTIMATION']['pose_estimate_method']
        self.imgsz=parser.getint('MODEL','infe_image_size')
        
        #naive config
        self.naive_scale = parser.getfloat('ESTIMATION','naive_scale')  
        self.naive_obj_x = parser.getfloat('ESTIMATION','naive_obj_x') 
        
    def cam_callback(self, data):
        
        # convert from ros to array
        np_arr = np.frombuffer(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        
    def obj_pose_estimation(self, class_name):
        if self.pose_estimate_method == ESTIMATION_TYPE['NAIVE']:
            print("naive estimation method")
                    
            #run 2d bbox detection
            pred = self.detector.predict(self.img)[0]
            self.detector.box_label(pred, self.img)
            
            self.naive_estimation(pred,class_name)
        elif self.pose_estimate_method == ESTIMATION_TYPE['LIDAR_SEG']:
            print("LIDAR_SEG estimation method")
            #run 2d bbox detection
            pred = self.detector.predict(self.img)[0]
            self.detector.box_label(pred, self.img)
            
            # self.lidar_seg_estimation(pred, lidar_pc)
        else:
            print("invalid estimation type, please use: \n", json.dumps(ESTIMATION_TYPE))
            
            
    def send_obj_tf(self,obj_pose, obj_tf,target_tf):
        
        self.br.sendTransform((obj_pose[0], obj_pose[1], 0.0),
                              (0.0,0.0,0.0,1.0),
                              rospy.Time.now(),
                              obj_tf,
                              target_tf
        )
            
if __name__== "__main__":
    estimate_c = object_pose_estimation()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        if estimate_c.img is None:
            print("waiting for image")
        else:
            estimate_c.obj_pose_estimation("obj1")
        rate.sleep()


    # image = cv2.imread('/home/robo-dell/Downloads/construction-safety.jpg',cv2.IMREAD_COLOR)
    # image = cv2.imread('/home/robo-dell/Downloads/2.jpg',cv2.IMREAD_COLOR)
    # print("oritin image shape is ", image.shape)
    # estimate_c.img = image
    # estimate_c.obj_pose_estimation("obj1")