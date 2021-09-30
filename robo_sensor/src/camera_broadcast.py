import rospy
import cv2
import json
import os, re
import subprocess
from camera_info_service import CameraInfoService
from sensor_msgs.msg import CompressedImage
from helpers import threaded
import signal, sys
import numpy as np

current_path = os.path.dirname(os.path.realpath(__file__))

class CameraBrodcast:
    def __init__(self, json_config_file=current_path+"../config/config.json"):
        self.json_file = json_config_file
        self.parm = None
        self.cam_frame_id = []
        self.device_id = []
        self.num_of_device = 0
        self.fps = 30
        self.rate = rospy.Rate(self.fps)
        self.resolution = []
        self.check_compliance = False
        self.quit = False
        self.set_of_publishers = {}
        self.set_of_cv_caps = {}
        self.set_of_cam_info = {}
        self.thread_process = []
        self.resolution_table = {"480p": (640, 480), "720p": (1280, 720), "1080p": (1920, 1080)}

        if not os.path.exists(json_config_file):
            raise FileNotFoundError("No JSON config file is found!")
        self.load_varaibles()
        three_channel_devices, _ = self.checkCameraID()
        if self.check_compliance:
            if len(three_channel_devices) != self.num_of_device:
                print("********** Found following deivces: ********** \n")
                print(three_channel_devices)
                raise AttributeError("Available cameras do not match with frame ID given.")
        print("********** Initiating robo_sensor -> camera node ********** \n")
        rospy.init_node('robo_sensor_camera', anonymous=True)
        self.setup_publishers()
        

    def load_varaibles(self):
        with open(self.json_file, "r") as f:
            self.pram = json.load(f)
        self.cam_frame_id = self.pram["cam_frame_id"]
        self.device_id = self.parm["device_id"]
        self.num_of_device = len(self.cam_frame_id)
        self.check_compliance = bool(self.parm["check_device_compliance"])
        self.resolution = self.parm["resolution"]
        self.fps = self.parm["fps"]
        assert len(self.resolution) == len(self.cam_frame_id), "Resolution configurations do not match camera frame IDs"
        
    def setup_publishers(self):
        for i in range(self.num_of_device):
            # Setting up publishers and cv caps
            resolution_w_h = self.resolution_table[self.resolution[i]]
            self.set_of_cv_caps[self.cam_frame_id[i]] = cv2.VideoCapture(self.device_id[i])
            self.set_of_cv_caps[self.cam_frame_id[i]].set(cv2.CAP_PROP_FRAME_WIDTH, resolution_w_h[0])
            self.set_of_cv_caps[self.cam_frame_id[i]].set(cv2.CAP_PROP_FRAME_HEIGHT, resolution_w_h[1])
            self.set_of_cv_caps[self.cam_frame_id[i]].set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.set_of_publishers[self.cam_frame_id[i]] = rospy.Publisher(self.cam_frame_id[i], CompressedImage, queue_size=1)
            self.set_of_cam_info[self.cam_frame_id[i]] = CameraInfoService(resolution=self.resolution[i], frmae_id=self.cam_frame_id[i])

    @threaded
    def update_loop(self, ind):
        cap = self.set_of_cv_caps[self.cam_frame_id[ind]]
        pub = self.set_of_publishers[self.cam_frame_id[ind]]
        cam_info = self.set_of_cam_info[self.cam_frame_id[ind]]
        while not (self.quit or rospy.is_shutdown()):
            _, frame = cap.read()
            if frame is not None:
                self.publish_image(pub, frame)
                cam_info.start_publish()
            self.rate.sleep()
            
            
    def main(self):
        signal.signal(signal.SIGINT, self.handelExit)
        for i in range(self.num_of_device):
            self.thread_process.append(self.update_loop(i))
        for th in self.thread_process:
            th.join()
        self.threads_process.clear()
        print("********** SIGINT detected, now exiting **********")
        for j in range(self.num_of_device):
            self.set_of_cv_caps[self.cam_frame_id[j]].release()
        signal.pause()
        sys.exit(0)


    def handelExit(self, sig, frame):
        self.quit = True


    @staticmethod
    def publish_image(pub, img):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()
        pub.publish(msg)

    @staticmethod    
    def checkCameraID():
        three_channel_devices = []
        gray_devices = []
        result = str(subprocess.run(['ls', '/dev'], stdout=subprocess.PIPE).stdout).split('\\n')
        for devs in result:
            if devs.startswith('video'):
                vid_number = [int(s) for s in re.findall(r'-?\d+\.?\d*', devs)][0]
                try:
                    cap = cv2.VideoCapture(vid_number)
                except:
                    pass
                if cap:
                    ret, frame = cap.read()
                    if frame is not None:
                        if len(frame.shape) == 3:
                            three_channel_devices.append(vid_number)
                        else:
                            gray_devices.append(vid_number)
        return three_channel_devices, gray_devices


if __name__ == "__main__":
    cam_brod = CameraBrodcast()