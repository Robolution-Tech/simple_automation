#!/usr/bin/env python

from maskrcnn_detect import MaskRcnn
from yolo5_detect import yolo5_detector
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from tf.transformations import quaternion_from_euler

import rospy
import os, sys
import numpy as np
import cv2
import json
import tf
import time

from helpers.helper_tools import timer, NumpyArrayEncoder
# from numba import njit

currentdir = os.path.dirname(os.path.realpath(__file__))

ESTIMATION_TYPE = {"NAIVE": "naive",
                   "LIDAR_SEG": "lidar_seg",
                    "MASKRCNN": "maskrcnn"}


class ObjectPoseEstimation():

    def __init__(self):
        rospy.init_node("object_pose_estimation_node", anonymous=True)
        self.br = tf.TransformBroadcaster()
        self.img = None
        self.uv = None

        
        ##################################################
        ###############ros param load#####################
        ##################################################
        
        ###camera parameters
        
        self.cam_topic_name= rospy.get_param('~sensor/cam_topic_name','/usb_cam/image_raw/compressed')
        self.lidar_topic_name= rospy.get_param('~sensor/lidar_topic_name','/livox/lidar_3GGDJ3400100831')
        
        cam_parms = rospy.get_param('~cam_lidar_seg/cam_fx_fy_cx_cy',[632.4, 644.17, 350.36, 237.68])
        cam_fx, cam_fy, cam_cx, cam_cy = cam_parms[0], cam_parms[1], cam_parms[2], cam_parms[3]
        cameraK = np.array([[cam_fx, 0, cam_cx],[0,cam_fy, cam_cy],[0,0,1]])
        cameraRT= np.asarray(rospy.get_param('~cam_lidar_seg/cam_RT',[[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0]]))
        self.cameraKRT = np.dot( cameraK, cameraRT )
        
        self.lidar_pose = np.asarray(rospy.get_param('~cam_lidar_seg/lidar_pose',[[1,0,0,0], [0,1,0,0], [0,0,1, 1]]))
        self.max_bbox_allowed = rospy.get_param('~max_bbox_allowed',1)
        
        
        #topic, frame
        self.processed_pose_frame = rospy.get_param(
            '/robo_param/frame_id/processed_pose_frame', "processed_pose_frame")
        
        ####pose estimation mode
        self.estimation_method = rospy.get_param('~pose_estimation/estimation_method','naive')
        print(self.estimation_method)
        #naive config
        
        self.naive_scale = rospy.get_param('~pose_estimation/naive_scale',1.2) # scale
        self.naive_obj_x = rospy.get_param('~pose_estimation/naive_obj_x',0.4) #width of object in m 
        
        
        #maskrcnn config
        self.maskrcnn_weight = currentdir+ rospy.get_param('~model/maskrcnn_weight','/maskrcnn_weights/model_final_f10217.pkl')
        
        
        if self.estimation_method == ESTIMATION_TYPE["LIDAR_SEG"] or self.estimation_method == ESTIMATION_TYPE["NAIVE"]:
            self.detector = yolo5_detector()
        elif self.estimation_method == ESTIMATION_TYPE["MASKRCNN"]:
            if os.path.exists(self.maskrcnn_weight):
                self.maskrcnn = MaskRcnn(self.maskrcnn_weight)
                self.mask = None
                self.maskrcnn_class_of_interest = rospy.get_param('~model/maskrcnn_class_of_interest',[0, 1])
                self.maskrcnn_accept_score = rospy.get_param('~model/maskrcnn_accept_score',0.7)
                self.maskrcnn_max_instance_allowed = rospy.get_param('~model/maskrcnn_max_instance_allowed',2)
            else:
                raise FileNotFoundError(
                    "No weight file for MaskRCNN is found!")

        self.cam_subscriber = rospy.Subscriber(
            self.cam_topic_name, CompressedImage, self.cam_callback,  queue_size=1)
        self.lidar_subscriber = rospy.Subscriber(
            self.lidar_topic_name, PointCloud2, self.lidar_callback)

    def naive_estimation(self, pred, class_name):
        """naive method to estimate object position"

        distance = obj_real_size(in m) /pixel_size(in m) * focal_length
        pixel_size = obj_pixel_y/img_pixel_y (reshaped) * img_pixel_y(origin) * n_pixel 
        Args:
            image ([type]): [description]
        """

        obj_num = len(pred)
        obj_pose_list = []

        # get image size

        # image = cv2.resize( self.img, self.detector.new_img_dim )

        img_x = self.detector.new_img_dim[0]
        img_y = self.detector.new_img_dim[1]

        # TODO: determin which obj is the right one to use
        # compute pose for each class_name obj
        for i in range(obj_num):

            # compute bbox size
            bbox = pred[i, 0:4]
            prob = pred[i, 4].item()
            class_id = self.detector.get_class_id(class_name)
            class_id_pred = pred[i, 5]
            if class_id == class_id_pred:
                start_point = (int(bbox[0]), int(bbox[1]))
                end_point = (int(bbox[2]), int(bbox[3]))

                len_x = end_point[0] - start_point[0]
                len_y = end_point[1] - start_point[1]
                
                #compute relative pose x,y
                pose_x = (img_x/ len_x + img_y/len_y) * self.naive_scale  * self.naive_obj_x
                # print("pose_x to obj", i, "is: ", pose_x)
                #pose_y  is equal to (bbox_mid - img_mid) / bbox_x*obj_x
                pose_y = ((end_point[0]-start_point[0]) - img_x/2 ) / len_x * self.naive_obj_x
                # print("pose_y to obj", i, "is: ", pose_y)
                
                #send tf
                self.send_obj_tf((pose_x,pose_y), "base_link", self.processed_pose_frame) #TODO: use dynamic frame id here for later use
                      
    def cam_callback(self, data):

        # convert from ros to array
        np_arr = np.frombuffer(data.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def obj_pose_estimation(self, class_name):

        if self.estimation_method == ESTIMATION_TYPE["NAIVE"]:
            # print("naive estimation method")
                    
            #run 2d bbox detection
            pred = self.detector.predict(self.img)[0]
            self.detector.box_label(pred, self.img)
            self.naive_estimation(pred, class_name)
            
        elif self.estimation_method == ESTIMATION_TYPE["LIDAR_SEG"]:
            # print("LIDAR_SEG estimation method")
            # run 2d bbox detection
            pred = self.detector.predict(self.img)[0]
            self.detector.box_label(pred, self.img)
            self.lidar_seg_estimation(pred, class_name)

        elif self.estimation_method == ESTIMATION_TYPE["MASKRCNN"]:
            self.maskrcnn_estimate()

        else:
            print(self.estimation_method)
            print(type(self.estimation_method))
            print("invalid estimation type, please choose estimation mode from: \n", ESTIMATION_TYPE)
            
            
    def send_obj_tf(self,obj_pose, obj_tf,target_tf):
        self.br.sendTransform((obj_pose[0], obj_pose[1], 0.0),
                              (0.0, 0.0, 0.0, 1.0),
                              rospy.Time.now(),
                              obj_tf,
                              target_tf
                              )

    @timer
    def lidar_seg_estimation(self, pred, class_name):
        if self.uv is None:
            return
        obj_num = len(pred)
        # print(obj_num)
        obj_pose_list = []
        img_x = self.detector.new_img_dim[0]
        img_y = self.detector.new_img_dim[1]
        counter = 0
        # for multiple bbox:
        obj_bbox = np.zeros((min(self.max_bbox_allowed, obj_num), 4))
        #TODO: determin which obj is the right one to use
        #compute pose for each class_name obj
        if obj_num > 0:
            for j in range(min(self.max_bbox_allowed, obj_num)):        
                #compute bbox size
                bbox = pred[j,0:4]
                prob = pred[j, 4].item()
                class_id = self.detector.get_class_id(class_name)
                class_id_pred = pred[j, 5]

                if class_id == class_id_pred:
                    start_point = (int(bbox[0]), int(bbox[1]))
                    end_point = (int(bbox[2]), int(bbox[3]))
                    obj_bbox[counter, :] = [int(bbox[0]), int(
                        bbox[1]), int(bbox[2]), int(bbox[3])]
                    counter += 1

            if counter > 0:
                num_of_obj = obj_bbox.shape[0]
                num_of_points = self.uv.shape[1]
                max_num_of_points_per_obj = rospy.get_param("~cam_lidar_seg/max_num_of_points_per_obj",4000)
                point_indices_every_obj = np.zeros(( num_of_obj, max_num_of_points_per_obj )).astype(np.int32)
                point_on_every_obj = np.zeros(( num_of_obj, max_num_of_points_per_obj, 2 ))
                point_counter_every_obj = np.zeros((num_of_obj)).astype(np.int32)

                # TODO: Multiple objects (with the same type) are detected?

                bbox_center = np.zeros((num_of_obj, 2))
                bbox_wh = np.zeros((num_of_obj, 2))
                bbox_wh_small = np.zeros((num_of_obj, 2))
                bbox_array = [start_point[0], start_point[1],
                              end_point[0], end_point[1]]
                bbox_center[:, 0] = (
                    (obj_bbox[:, 1] + obj_bbox[:, 3])/2.0).astype(np.int32)
                bbox_center[:, 1] = (
                    (obj_bbox[:, 0] + obj_bbox[:, 2])/2.0).astype(np.int32)
                bbox_wh[:, 0] = obj_bbox[:, 3] - obj_bbox[:, 1]
                bbox_wh[:, 1] = obj_bbox[:, 2] - obj_bbox[:, 0]
                bbox_trim_edge = (bbox_wh/6).astype(np.int32)
                obj_box_trimmed_array = np.zeros_like(obj_bbox)
                obj_box_trimmed_array[:, 0] = obj_bbox[:,
                                                       0] + bbox_trim_edge[:, 1]
                obj_box_trimmed_array[:, 1] = obj_bbox[:,
                                                       1] - bbox_trim_edge[:, 1]
                obj_box_trimmed_array[:, 2] = obj_bbox[:,
                                                       2] + bbox_trim_edge[:, 0]
                obj_box_trimmed_array[:, 3] = obj_bbox[:,
                                                       3] - bbox_trim_edge[:, 0]

                for j in range(num_of_obj):
                    original_bbox_inds = np.where((self.uv[1] >= obj_bbox[j, 0]) & (self.uv[1] <= obj_bbox[j, 2]) & (
                        self.uv[0] >= obj_bbox[j, 1]) & (self.uv[0] <= obj_bbox[j, 3]))
                    inds = np.where((self.uv[1] >= obj_box_trimmed_array[j, 0]) & (self.uv[1] <= obj_box_trimmed_array[j, 2]) & (
                        self.uv[0] >= obj_box_trimmed_array[j, 1]) & (self.uv[0] <= obj_box_trimmed_array[j, 3]))
                    print('cropped  bbox points: ', inds[0].shape)
                    print('original bbox points: ',
                          original_bbox_inds[0].shape)
                    length_of_points = inds[0].shape[0]
                    point_indices_every_obj[j, :length_of_points] = inds[0]

                people_xyz_array = np.zeros((num_of_obj, 3))

                for jj in range(num_of_obj):
                    point_indices_this_obj = point_indices_every_obj[jj][point_indices_every_obj[jj] != 0]
                    # [:point_counter_every_person[ii]]
                    points_xyz_this_obj = self.p_xyz[:, point_indices_this_obj]
                    points_xyz_this_obj_to_baselink = np.dot(
                        self.lidar_pose, points_xyz_this_obj)
                    # people_xyz_array[ii, :] = points_xyz_this_obj_to_baselink
                    people_xyz_array[jj, :] = [np.average(points_xyz_this_obj_to_baselink[0, :]), np.average(
                        points_xyz_this_obj_to_baselink[1, :]), np.average(points_xyz_this_obj_to_baselink[2, :])]

                pose_x = np.average(people_xyz_array[:, 0])
                pose_y = np.average(people_xyz_array[:, 1])
                print("Sending pose: {}, {}".format(pose_x, pose_y))
                self.send_obj_tf((pose_x, pose_y), "base_link", self.processed_pose_frame)

    def maskrcnn_estimate(self):
        if self.img is not None:
            if len(self.maskrcnn_class_of_interest) > 1:
                raise ValueError(
                    "Only one class is supported. Change config file to fix this")
                # TODO: How to handle multiple objects?
            else:
                vis_img, self.mask = self.maskrcnn.inference(
                    self.img, self.maskrcnn_class_of_interest, self.maskrcnn_max_instance_allowed, self.maskrcnn_accept_score)

                if self.mask is not None and self.uv is not None:  # object is detected.
                    # First filter: check if point is within the frame: uv[0] : x; uv[1] : y
                    cv2.imshow("aa", self.mask)
                    cv2.waitKey(10)
                    img_size = self.img.shape[:2]
                    # print(img_size)
                    uv_copy = self.uv.copy()
                    uv_within_frame_ind = np.where((uv_copy[1] > 0) & (
                        uv_copy[1] < img_size[0]) & (uv_copy[0] > 0) & (uv_copy[0] < img_size[1]))[0]
                    ok_list = []
                    # Second filter: check if point in the mask is 1(true) or 0(false)
                    for point_ind in uv_within_frame_ind.tolist():
                        # print(uv[:, point_ind])
                        u, v = uv_copy[:,
                                       point_ind][0], uv_copy[:, point_ind][1]
                        # print(u, v)
                        if self.mask[v, u]:
                            ok_list.append(point_ind)
                    # [:point_counter_every_person[ii]]
                    # print(ok_list)
                    if len(ok_list) != 0:
                        points_xyz_this_obj = self.p_xyz[:, np.array(ok_list)]
                        points_xyz_this_obj_to_baselink = np.dot(
                            self.lidar_pose, points_xyz_this_obj)
                        final_x = np.average(points_xyz_this_obj_to_baselink[0])
                        final_y = np.average(points_xyz_this_obj_to_baselink[1])
                        print("Sending pose: {}, {}".format(final_x, final_y))
                        self.send_obj_tf((final_x, final_y), "base_link",
                                        self.processed_pose_frame)

    @staticmethod
    def config_json(config_file_path):
        with open(config_file_path, "r") as f:
            return json.load(f)

    def lidar_callback(self, msg):
        self.p_xyz = np.ones((4, rospy.get_param('~cam_lidar_seg/number_of_points',9984)))
        pt_count = 0
        t1 = time.time()
        for point in sensor_msgs.point_cloud2.read_points(msg, skip_nans=True):
            self.p_xyz[0:3, pt_count] = point[0:3]
            pt_count += 1
        t = int((time.time() - t1)*1000)
        # print('pppc2 :', t)
        uvw = np.dot( self.cameraKRT, self.p_xyz )
        uvw[2][uvw[2]==0.0] = 0.01
        uvw /= uvw[2]  
        uvw = uvw.astype(np.int32)
        self.uv = uvw[0:2]

if __name__== "__main__":
    estimate_c = ObjectPoseEstimation()
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
