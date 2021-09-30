#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
from tf.transformations import quaternion_from_euler


import os
import rospy, tf
import math
import configparser

currentdir = os.path.dirname( os.path.realpath(__file__) )

def coord_transform_rotation(x,y,theta):
    x_out = math.cos(theta) * x + math.sin(theta) * y
    y_out = -math.sin(theta) * x + math.cos(theta) * y
    return [x_out, y_out]

def add_target_pose_offset(pose,offset_x, offset_y):
    pose_x = pose[0] + offset_x
    pose_y = pose[1] + offset_y

    return [pose_x, pose_y]



class target_pose_process():
    
    def __init__(self):
        self.parse_variables(currentdir + "/detection_config.ini") 
        self.equipment_gps = None
        self.target_gps = None
        self.robo_decision_system_state = None
        
        rospy.init_node("target_pose_process_node", anonymous=True)
        self.equipment_gps_subscriber = rospy.Subscriber("estimated_lat_lon_dir", NavSatFix, self.equipment_gps_callback,  queue_size = 1)
        self.target_gps_subscriber = rospy.Subscriber("target_gps",NavSatFix, self.target_gps_callback,  queue_size = 1)
        self.processed_pose_publisher = rospy.Publisher('processed_pose',PoseStamped,queue_size=10)
        
        #sub decision tree state
        self.decision_tree_state_subscriber = rospy.Subscriber("robo_decision_system_state", Int8, self.robo_decision_system_state_callback)
        self.tf_listener = tf.TransformListener()
        
        
        self.processed_pose = PoseStamped()
        
    def equipment_gps_callback(self, data):
        self.equipment_gps = data
        
    def target_gps_callback(self, data):
        self.target_gps = data
        
    def robo_decision_system_state_callback(self,data):
        self.robo_decision_system_state = data    
    
    def send_pose(self):
        
        if self.target_gps is not None:
            #compute target relative pose
            #TODO update estimated_lat_lon_dir msg type to customized msg
            
            #global relative pose
            global_relative_pose_lon = self.target_gps.longitude - self.equipment_gps.longitude
            global_relative_pose_lat = self.target_gps.latitude - self.equipment_gps.latitude
            #TODO multiply by a scale to transform lat,lon to meter
            #transfer to relative relative to equipment 
            theta = 0 # TODO get from new msg type
            pose_trans = coord_transform_rotation(global_relative_pose_lat,global_relative_pose_lon,theta)
            
            #angle compute
            global_relatie_angle = math.atan2(global_relative_pose_lon,global_relative_pose_lat)
            angle_trans = theta - global_relatie_angle
            
            pose_trans = add_target_pose_offset(pose_trans,self.offset_x, self.offset_y)
            self.processed_pose.header.stamp = rospy.Time.now()
            self.processed_pose.header.frame_id = "obj1"
            
            self.processed_pose.pose.position.x = pose_trans[0]
            self.processed_pose.pose.position.y = pose_trans[1]
            self.processed_pose.pose.position.z = 0
            
            q = quaternion_from_euler(0, 0, angle_trans)
            self.processed_pose.pose.orientation = q
            
            # print(self.processed_pose)
            self.processed_pose_publisher.publish(self.processed_pose)
            return True
        else:
            try:
                (pose_trans, rot_trans) = self.tf_listener.lookupTransform('/obj1','/base_link', rospy.Time(0))
                # print(pose_trans)
                # print(rot_trans)
                pose_offset = add_target_pose_offset(pose_trans,self.offset_x, self.offset_y)
                self.processed_pose.header.stamp = rospy.Time.now()
                self.processed_pose.header.frame_id = "obj1"
                
                self.processed_pose.pose.position.x = pose_offset[0]
                self.processed_pose.pose.position.y = pose_offset[1]
                self.processed_pose.pose.position.z = pose_trans[2]
                
                self.processed_pose.pose.orientation.x = rot_trans[0]
                self.processed_pose.pose.orientation.y = rot_trans[1]
                self.processed_pose.pose.orientation.z = rot_trans[2]
                self.processed_pose.pose.orientation.w = rot_trans[3]

                # print(self.processed_pose)
                self.processed_pose_publisher.publish(self.processed_pose)                
                return True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
                rospy.logdebug("object pose estimation, tf listener ", err)
            
        return False
    
    
    def get_processed_pose(self):
        return self.processed_pose

    def parse_variables(self, config_file):
        parser = configparser.ConfigParser()
        parser.read(config_file)

        self.offset_x = parser.getfloat('TARGET_POSE','offset_x')  
        self.offset_y = parser.getfloat('TARGET_POSE','offset_y')  
       
        
        

if __name__== "__main__":
    pose_p = target_pose_process()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pose_p.send_pose()
        rate.sleep()