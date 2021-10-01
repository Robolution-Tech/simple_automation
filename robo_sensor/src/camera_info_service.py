import yaml
import os
import rospy
from sensor_msgs.msg import CameraInfo

current_path = os.path.dirname(os.path.realpath(__file__))
supported_resolution = ["1080p", "720p", "480p"]
corresponding_yaml_file = {"1080p": "1920_1080.yaml", "720p": "1280_720.yaml", "480p": "640_480.yaml"}

class CameraInfoService:
    def __init__(self, resolution="720p", frmae_id="usb_cam_0"):
        self.cam_para = None
        self.cam_info = CameraInfo()
        self.cam_frame_id = frmae_id
        self.cam_info_pub = rospy.Publisher(self.cam_frame_id + "/CameraInfo", CameraInfo, queue_size=1)
        if resolution not in supported_resolution:
            raise ValueError("Resolution not supported!")
        else:
            yaml_path = os.path.join(current_path, "../config/" + corresponding_yaml_file[resolution])
            if os.path.exists(yaml_path):
                self.load_camera_para(yaml_path)
            else:
                raise FileNotFoundError("No camera yaml file is found!")
        
    def load_camera_para(self, yaml_path):
        with open(yaml_path, "r") as f:
            self.cam_para = yaml.load(f, Loader=yaml.CLoader)
        if self.cam_para is not None:
            self.setup_cam_header()
    
    def setup_cam_header(self):
        self.cam_info.header.frame_id = self.cam_frame_id
        self.cam_info.height = self.cam_para["image_height"]
        self.cam_info.width = self.cam_para["image_width"]
        self.cam_info.distortion_model = self.cam_para["distortion_model"]
        self.cam_info.D = self.cam_para["distortion_coefficients"]["data"]
        self.cam_info.K = self.cam_para["camera_matrix"]["data"]
        self.cam_info.R = self.cam_para["rectification_matrix"]["data"]
        self.cam_info.P = self.cam_para["projection_matrix"]["data"]
    
    def start_publish(self):
        # while not rospy.is_shutdown():
        #     self.cam_info.header.stamp = rospy.Time.now()
        #     self.cam_info_pub.publish(self.cam_info)
        #     self.rate.sleep()
        self.cam_info.header.stamp = rospy.Time.now()
        self.cam_info_pub.publish(self.cam_info)

if __name__ == "__main__":
    cam_serv = CameraInfoService()

