#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from std_msgs.msg import Int8
from tf.transformations import quaternion_from_euler, euler_from_quaternion


import os
import rospy
import tf
import math

currentdir = os.path.dirname(os.path.realpath(__file__))


def latlon2meter(ll, ll_ref):
    '''
    ll: (lat,lon) or [lat,lon]
    ll_ref: same format as above, just another point

    lat, lat_ref --> dy
    lon, lon_ref --> dx 
    '''
    dlat = (ll[0] - ll_ref[0]) * math.pi / 180.0
    dlon = (ll[1] - ll_ref[1]) * math.pi / 180.0
    # print(dlat, dlon)
    earth_radius = 6378100     # meter
    earth_peri = 40074784.251  # meter
    dy = (dlat/(math.pi/2)) * (earth_peri/4)  # meter from ref to ll
    dx = earth_radius * math.cos(ll[0] * math.pi / 180.0) * dlon
    return dx, dy


def coord_transform_rotation(x, y, theta):
    x_out = math.cos(theta) * x + math.sin(theta) * y
    y_out = -math.sin(theta) * x + math.cos(theta) * y
    return [x_out, y_out]


def add_vector_offset(pose, scale):
    angle = math.atan2(pose[1], pose[0])
    pose_x = pose[0] + math.cos(angle) * scale
    pose_y = pose[1] + math.sin(angle) * scale

    return [pose_x, pose_y]


class target_pose_process():

    def __init__(self):
        rospy.init_node("target_pose_process_node", anonymous=True)

        self.dynamic_offset = rospy.get_param('~offset/dynamic_offset', False)
        self.pose_offset_x = rospy.get_param('~offset/pose_offset_x', 1.0)
        self.pose_offset_y = rospy.get_param('~offset/pose_offset_y', 1.0)
        self.dynamic_scale = rospy.get_param('~offset/dynamic_scale', 1.0)
        self.gps_to_meter_scale = rospy.get_param(
            '~gps/gps_to_meter_scale', 1.0)

        self.equipment_gps_location_topic = rospy.get_param(
            '/robo_param/topic_names/equipment_gps_location_topic', "equipment_gps")
        self.equipment_imu_topic = rospy.get_param(
            '/robo_param/topic_names/equipment_imu_topic', "equipment_imu")
        self.user_input_target_gps_topic = rospy.get_param(
            '/robo_param/topic_names/user_input_target_gps_topic', "target_gps")
        self.processed_pose_topic = rospy.get_param(
            '/robo_param/topic_names/processed_pose_topic', "processed_pose")
        self.robo_decision_system_state_topic = rospy.get_param(
            '/robo_param/topic_names/robo_decision_system_state_topic', "robo_decision_system_state")
        self.processed_pose_frame = rospy.get_param(
            '/robo_param/frame_id/processed_pose_frame', "processed_pose_frame")


        self.equipment_gps = None
        self.target_gps = None
        self.robo_decision_system_state = None
        self.imu = None

        self.equipment_gps_subscriber = rospy.Subscriber(
            self.equipment_gps_location_topic, NavSatFix, self.equipment_gps_callback,  queue_size=1)
        self.equipment_imu_subscriber = rospy.Subscriber(
            self.equipment_imu_topic, Imu, self.equipment_imu_callback,  queue_size=10)
        self.target_gps_subscriber = rospy.Subscriber(
            "target_gps", NavSatFix, self.target_gps_callback,  queue_size=1)
        self.processed_pose_publisher = rospy.Publisher(
            'processed_pose', PoseStamped, queue_size=10)

        # sub decision tree state
        self.decision_tree_state_subscriber = rospy.Subscriber(
            "robo_decision_system_state", Int8, self.robo_decision_system_state_callback)
        self.tf_listener = tf.TransformListener()

        self.processed_pose = PoseStamped()

    def equipment_gps_callback(self, data):
        self.equipment_gps = data

    def equipment_imu_callback(self, data):
        self.imu = data

    def target_gps_callback(self, data):
        self.target_gps = data

    def robo_decision_system_state_callback(self, data):
        self.robo_decision_system_state = data

    def send_pose(self):

        if self.target_gps is not None:
            # compute target relative pose
            # TODO update estimated_lat_lon_dir msg type to customized msg

            # global relative pose
            global_relative_pose_lon = (
                self.target_gps.longitude - self.equipment_gps.longitude) * self.gps_to_meter_scale
            global_relative_pose_lat = (
                self.target_gps.latitude - self.equipment_gps.latitude) * self.gps_to_meter_scale

            global_relative_pose_lon, global_relative_pose_lat = latlon2meter([self.target_gps.latitude, self.target_gps.longitude], [
                                                                              self.equipment_gps.latitude, self.equipment_gps.longitude])

            # TODO multiply by a scale to transform lat,lon to meter
            # transfer to relative relative to equipment
            imu_orientation = euler_from_quaternion(self.imu.orientation)
            theta = imu_orientation[2]  # TODO get from new msg type
            pose_trans = coord_transform_rotation(
                global_relative_pose_lat, global_relative_pose_lon, theta)

            # angle compute
            global_relatie_angle = math.atan2(
                global_relative_pose_lon, global_relative_pose_lat)
            angle_trans = theta - global_relatie_angle

            pose_trans = self.add_target_pose_offset(pose_trans)
            self.processed_pose.header.stamp = rospy.Time.now()
            self.processed_pose.header.frame_id = self.processed_pose_frame

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
                (pose_trans, rot_trans) = self.tf_listener.lookupTransform(
                    self.processed_pose_frame, '/base_link', rospy.Time(0))
                # print(pose_trans)
                # print(rot_trans)
                pose_offset = self.add_target_pose_offset(pose_trans)
                self.processed_pose.header.stamp = rospy.Time.now()
                self.processed_pose.header.frame_id = self.processed_pose_frame

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

    def add_target_pose_offset(self, pose):
        print(self.dynamic_offset)
        print("pose is", pose)
        if self.dynamic_offset == True:
            processed_pose = add_vector_offset(pose, self.dynamic_scale)
        else:
            pose_x = pose[0] + self.pose_offset_x
            pose_y = pose[1] + self.pose_offset_y

            processed_pose = [pose_x, pose_y]

        print("processed pose is", processed_pose)
        return processed_pose


if __name__ == "__main__":
    pose_p = target_pose_process()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pose_p.send_pose()
        rate.sleep()
