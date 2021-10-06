#!/usr/bin/env python
import json, cv2, time, os, math
import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import NavSatFix
import numpy as np
import requests, shutil
from os import listdir
from os.path import isfile, join

print('CV version: {}'.format(cv2. __version__))
currentdir = os.path.dirname(os.path.realpath(__file__))
class targetSelection():
    def __init__(self):

        rospy.init_node('node_user_ui')

        user_input_target_gps_topic = rospy.get_param('/robo_param/topic_names/user_input_target_gps_topic','target_gps')
        robo_decision_user_command = rospy.get_param('/robo_param/topic_names/robo_decision_user_command','user_command')
        
        self.user_pub = rospy.Publisher(robo_decision_user_command, Int8, queue_size=1)
        self.target_gps_pub = rospy.Publisher(user_input_target_gps_topic, NavSatFix, queue_size=1)

        self.latlon = [53.312726, -113.580793]

        self.map_tiles_folder = currentdir + '/maps/'

        self.visual_img_width = 800
        self.visual_img_height = 800

        # load config 
        general_config_file = open(currentdir+'/lat_lon_global_config.json', 'r')
        general_config = json.load(general_config_file)
        general_config_file.close()

        self.image_resolution = general_config['image_resolution']
        self.zoom_level = general_config['zoom_level']
        self.api_key = 'pk.eyJ1IjoieGJ3MTI2NiIsImEiOiJja3BqOHI1cTAxajNoMnZwM2s3ejJ6NjY1In0.teLIW7gG945RkmgJDWWJWQ'

        self.tile_num = 3  # use odd number 

        # load global tile info
        all_tile_info_file = open(currentdir+'/global_tile_coord.json','r')
        all_tile_info_file_content = json.load(all_tile_info_file)
        all_tile_info_file.close()

        self.all_tile_center_lats = all_tile_info_file_content["lat_centers_list"]
        self.all_tile_center_lons = all_tile_info_file_content["lon_centers_list"]

        self.all_tile_corner_lats = all_tile_info_file_content["lat_tile_range"]
        self.all_tile_corner_lons = all_tile_info_file_content["lon_tile_range"]


        location_tile_index = self.LatLonToTileIndex( self.latlon[0], self.latlon[1] )

        print(location_tile_index)

        self.complete_image_original_large  = self.AssembleTiles( location_tile_index[0], location_tile_index[1], self.tile_num, self.tile_num )

        self.complete_image = cv2.resize(self.complete_image_original_large.copy(), (self.visual_img_width, self.visual_img_height) )
        self.complete_image_backup = self.complete_image.copy()
        # self.textbg = np.ones((160, 800, 3)).astype(np.uint8) * 255
        self.textbg_init()
        self.complete_image_with_text = np.vstack( ( self.complete_image.copy(),  self.textbg  ) )

        self.select_roi()

        self.select_target_location()

        self.select_action()



    # =============================================================
    # clear the text bg region
    # =============================================================
    def textbg_init(self):
        self.textbg = np.ones((160, self.visual_img_width, 3)).astype(np.uint8) * 255


    # =============================================================
    # one of main functions, for ROI selection 
    # =============================================================
    def selection_img_init(self):
        self.textbg_init()
        map_part = self.complete_image_backup.copy()
        self.complete_image_with_text = np.vstack( ( map_part,  self.textbg  ) )




    # =============================================================
    # mouse event handle, for ROI selection 
    # =============================================================
    def click_and_crop( self, event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting (x, y) coordinates
        if event == cv2.EVENT_LBUTTONDOWN:
            # cv2.circle(self.complete_image_with_text, (x,y), 5, (0,255,0), -1)
            if 0 <= x <= self.visual_img_width and 0 <= y <= self.visual_img_height:
                self.roi_selection.append( [x,y] )

            if self.button_roi_restart_boundary[0][0] < x < self.button_roi_restart_boundary[1][0] and self.button_roi_restart_boundary[0][1] < y < self.button_roi_restart_boundary[1][1] :
                self.roi_restart_button_pressed = True 
                print('Restart pressed ')
            if len(self.roi_selection) == 2:
                if self.button_roi_set_boundary[0][0] < x < self.button_roi_set_boundary[1][0] and self.button_roi_set_boundary[0][1] < y < self.button_roi_set_boundary[1][1] :
                    self.roi_set_button_pressed = True 
                    print('Set pressed ')


    # =============================================================
    # one of main functions, for ROI selection 
    # =============================================================
    def select_roi(self):

        self.roi_selection = []

        font = cv2.FONT_HERSHEY_SIMPLEX

        button_width  = 130
        button_height = 90
        button_top_offset = 60

        roi_restart_left = 630
        roi_set_left  =480


        self.button_roi_restart_boundary = [(roi_restart_left, self.visual_img_height+button_top_offset),(roi_restart_left+button_width, self.visual_img_height+button_top_offset+button_height)]
        self.button_roi_set_boundary     = [(roi_set_left, self.visual_img_height+button_top_offset)    ,(roi_set_left+button_width , self.visual_img_height+button_top_offset+button_height)]
        
        self.roi_restart_button_pressed = False
        self.roi_set_button_pressed = False

        self.complete_image_with_text_backup = self.complete_image_with_text.copy()
        cv2.namedWindow("map")
        cv2.setMouseCallback("map", self.click_and_crop)

        # while True:
        while self.roi_set_button_pressed == False:
            # display the image and wait for a keypress
            cv2.imshow("map", self.complete_image_with_text)
            key = cv2.waitKey(1) & 0xFF

            if self.roi_restart_button_pressed == True:
                # self.complete_image_with_text = self.complete_image_with_text_backup.copy()
                # self.textbg_init()
                self.selection_img_init()
                self.complete_image_with_text = np.vstack( ( self.complete_image_backup,  self.textbg  ) )
                cv2.rectangle( self.complete_image_with_text, self.button_roi_restart_boundary[0], self.button_roi_restart_boundary[1], (0,255,0), 5 )
                cv2.rectangle( self.complete_image_with_text, self.button_roi_restart_boundary[0], self.button_roi_restart_boundary[1], (100,255,100), -1 )
                UIstring = 'Restart'
                cv2.putText(self.complete_image_with_text, UIstring, (self.button_roi_restart_boundary[0][0]+10, self.button_roi_restart_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)

                self.roi_selection = []
                self.roi_restart_button_pressed = False
                self.roi_set_button_pressed = False

            else:
                self.selection_img_init()

                cv2.rectangle( self.complete_image_with_text, self.button_roi_restart_boundary[0], self.button_roi_restart_boundary[1], (0,255,0), 5 )
                cv2.rectangle( self.complete_image_with_text, self.button_roi_restart_boundary[0], self.button_roi_restart_boundary[1], (100,255,100), -1 )
                UIstring = 'Restart'
                cv2.putText(self.complete_image_with_text, UIstring, (self.button_roi_restart_boundary[0][0]+10, self.button_roi_restart_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)

                if len(self.roi_selection) < 2:
                    
                    UIstring = 'Select the region of the site.'
                    cv2.putText(self.complete_image_with_text, UIstring, (20,840), font, 1, (255, 100, 60), 2)

                    UIstring = 'Please select 2 points, now have {}'.format(len(self.roi_selection))
                    cv2.putText(self.complete_image_with_text, UIstring, (20,880), font, 1, (255, 0, 0), 2)

                    if len(self.roi_selection) > 0:
                        for spasdsadpks in self.roi_selection:
                            cv2.circle(self.complete_image_with_text, ( spasdsadpks[0] , spasdsadpks[1] ), 5, (0,255,0), -1)

                if len(self.roi_selection) > 2:
                    UIstring = 'Please select 2 points only, now have {}.'.format(len(self.roi_selection))
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(self.complete_image_with_text, UIstring, (20,840), font, 1, (255, 0, 0), 2)
                    UIstring = 'Press Restart.'
                    cv2.putText(self.complete_image_with_text, UIstring, (20,880), font, 1, (25, 20, 255), 2)

                if len(self.roi_selection) == 2:
                    UIstring = 'Good -> Press Set'
                    roi_up = min(self.roi_selection[0][1], self.roi_selection[1][1])
                    roi_bt = max(self.roi_selection[0][1], self.roi_selection[1][1])
                    roi_lf = min(self.roi_selection[0][0], self.roi_selection[1][0])
                    roi_rt = max(self.roi_selection[0][0], self.roi_selection[1][0])
                    cv2.rectangle( self.complete_image_with_text, (roi_rt, roi_up), (roi_lf, roi_bt), (0,255,0), 3 )
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(self.complete_image_with_text, UIstring, (20,840), font, 1, (255, 0, 0), 2)
                    UIstring = 'Reselect-> Press Restart'
                    cv2.putText(self.complete_image_with_text, UIstring, (20,880), font, 1, (255, 0, 0), 2)

                    cv2.rectangle( self.complete_image_with_text, self.button_roi_set_boundary[0], self.button_roi_set_boundary[1], (0,255,0), 5 )
                    cv2.rectangle( self.complete_image_with_text, self.button_roi_set_boundary[0], self.button_roi_set_boundary[1], (100,255,100), -1 )
                    UIstring = 'Set'
                    cv2.putText(self.complete_image_with_text, UIstring, (self.button_roi_set_boundary[0][0]+10, self.button_roi_set_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)

       

        self.complete_image = self.complete_image_backup.copy()
        roi_up = min(self.roi_selection[0][1], self.roi_selection[1][1])
        roi_bt = max(self.roi_selection[0][1], self.roi_selection[1][1])
        roi_lf = min(self.roi_selection[0][0], self.roi_selection[1][0])
        roi_rt = max(self.roi_selection[0][0], self.roi_selection[1][0])
        self.complete_image = self.complete_image[ roi_up:roi_bt, roi_lf:roi_rt ]

        new_latlon_topleft = self.pix2latlon(  (roi_lf, roi_up)  )
        new_latlon_botright = self.pix2latlon(  (roi_rt, roi_bt)  )

        print( new_latlon_topleft)
        print(new_latlon_botright)

        self.current_lon_range = [ new_latlon_topleft[1], new_latlon_botright[1]]
        self.current_lat_range = [ new_latlon_topleft[0], new_latlon_botright[0]]

        ### update the image for visual, for the next step 
        roi_up_in_original = int( float(roi_up)/self.visual_img_height * (self.tile_num * self.image_resolution) )
        roi_bt_in_original = int( float(roi_bt)/self.visual_img_height * (self.tile_num * self.image_resolution) )
        roi_lt_in_original = int( float(roi_lf)/self.visual_img_height * (self.tile_num * self.image_resolution) )
        roi_rt_in_original = int( float(roi_rt)/self.visual_img_height * (self.tile_num * self.image_resolution) )

        roi_map = self.complete_image_original_large[roi_up_in_original:roi_bt_in_original, roi_lt_in_original:roi_rt_in_original]

        neww = roi_map.shape[1]
        newh = roi_map.shape[0]
        height = int(self.visual_img_width / float(neww) * newh)
        self.complete_image = cv2.resize( roi_map, (self.visual_img_width, height ) )

        self.visual_img_height = height

        cv2.destroyWindow("map") 


    # =============================================================
    # one of main functions, for target locations selection 
    # =============================================================
    def select_target_location(self):
        self.target_selection = []

        self.target_number_limit = 1

        font = cv2.FONT_HERSHEY_SIMPLEX

        button_width  = 130
        button_height = 90
        button_top_offset = 60

        target_restart_left = 630
        target_set_left  =480


        self.button_target_restart_boundary = [(target_restart_left, self.visual_img_height+button_top_offset),(target_restart_left+button_width, self.visual_img_height+button_top_offset+button_height)]
        self.button_target_set_boundary     = [(target_set_left, self.visual_img_height+button_top_offset)    ,(target_set_left+button_width , self.visual_img_height+button_top_offset+button_height)]
        
        self.target_restart_button_pressed = False
        self.target_set_button_pressed = False

        self.complete_image_backup = self.complete_image.copy()
        self.complete_image_with_text_backup = self.complete_image_with_text.copy()

        self.targetlatlon = NavSatFix()
        
        cv2.namedWindow("Task Assign")
        cv2.setMouseCallback("Task Assign", self.click_for_target)

        # while True:
        while self.target_set_button_pressed == False:
            # display the image and wait for a keypress

            font = cv2.FONT_HERSHEY_SIMPLEX

            # UIstring = 'Select target locations in sequence.'
            UIstring = 'Select target location. Have {}, need {} more'.format( len(self.target_selection) , self.target_number_limit-len(self.target_selection))
            cv2.putText(self.complete_image_with_text, UIstring, (20, self.visual_img_height+40 ), font, 1, (255, 100, 60), 2)

            cv2.imshow("Task Assign", self.complete_image_with_text)
            key = cv2.waitKey(1) & 0xFF

            if self.target_restart_button_pressed == True:
                # self.complete_image_with_text = self.complete_image_with_text_backup.copy()
                # self.textbg_init()
                self.selection_img_init()
                self.complete_image_with_text = np.vstack( ( self.complete_image_backup,  self.textbg  ) )
                cv2.rectangle( self.complete_image_with_text, self.button_target_restart_boundary[0], self.button_target_restart_boundary[1], (0,255,0), 5 )
                cv2.rectangle( self.complete_image_with_text, self.button_target_restart_boundary[0], self.button_target_restart_boundary[1], (100,255,100), -1 )
                UIstring = 'Restart'
                cv2.putText(self.complete_image_with_text, UIstring, (self.button_target_restart_boundary[0][0]+10, self.button_target_restart_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)

                self.target_selection = []
                self.target_restart_button_pressed = False
                self.target_set_button_pressed = False

            else:
                self.selection_img_init()

                if len(self.target_selection) == self.target_number_limit:

                    cv2.rectangle( self.complete_image_with_text, self.button_target_set_boundary[0], self.button_target_set_boundary[1], (0,255,0), 5 )
                    cv2.rectangle( self.complete_image_with_text, self.button_target_set_boundary[0], self.button_target_set_boundary[1], (100,255,100), -1 )
                    UIstring = 'Set'
                    cv2.putText(self.complete_image_with_text, UIstring, (self.button_target_set_boundary[0][0]+10, self.button_target_set_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)


                cv2.rectangle( self.complete_image_with_text, self.button_target_restart_boundary[0], self.button_target_restart_boundary[1], (0,255,0), 5 )
                cv2.rectangle( self.complete_image_with_text, self.button_target_restart_boundary[0], self.button_target_restart_boundary[1], (100,255,100), -1 )
                UIstring = 'Restart'
                cv2.putText(self.complete_image_with_text, UIstring, (self.button_target_restart_boundary[0][0]+10, self.button_target_restart_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)


                if len(self.target_selection) > 0:
                        for spasdsadpks in self.target_selection:
                            cv2.circle(self.complete_image_with_text, ( spasdsadpks[0] , spasdsadpks[1] ), 5, (0,255,0), -1)

                UIstring = 'Select target location. Have {}, need {} more'.format( len(self.target_selection) , self.target_number_limit-len(self.target_selection))
                cv2.putText(self.complete_image_with_text, UIstring, (20, self.visual_img_height+40 ), font, 1, (255, 100, 60), 2)

                UIstring = 'Good -> Press Set'
                cv2.putText(self.complete_image_with_text, UIstring, (20, self.visual_img_height+80 ), font, 1, (255, 0, 0), 2)

                UIstring = 'Reselect-> Press Restart'
                cv2.putText(self.complete_image_with_text, UIstring, (20, self.visual_img_height+120 ), font, 1, (255, 0, 0), 2)

        print(self.target_selection)
        self.targetlatlon.latitude = self.target_selection[0][2]
        self.targetlatlon.longitude = self.target_selection[0][3]











    # =============================================================
    # mouse event handle, for target locaton selections 
    # =============================================================
    def click_for_target(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if 0 <= x <= self.visual_img_width and 0 <= y <= self.visual_img_height:
                (lat, lon) = self.pix2latlon((x,y))
                self.target_selection.append( [x,y, lat, lon] )
                while len(self.target_selection) > self.target_number_limit:
                    self.target_selection.pop(0)

            if self.button_target_restart_boundary[0][0] < x < self.button_target_restart_boundary[1][0] and self.button_target_restart_boundary[0][1] < y < self.button_target_restart_boundary[1][1] :
                self.target_restart_button_pressed = True 
                print('Restart pressed ')
            if len(self.target_selection) == self.target_number_limit:
                if self.button_target_set_boundary[0][0] < x < self.button_target_set_boundary[1][0] and self.button_target_set_boundary[0][1] < y < self.button_target_set_boundary[1][1] :
                    self.target_set_button_pressed = True 
                    print('Set pressed ')




    # =============================================================
    # one of main functions, for user-wanted action update 
    # =============================================================
    def select_action(self):
        self.action = -1

        self.complete_image_backup = self.complete_image.copy()
        self.complete_image_with_text_backup = self.complete_image_with_text.copy()

        button_width  = 100
        button_height = 100
        button_top_offset = 30

        start_left = 30
        home_left  = 230
        abort_left = 430

        app_exit_left = 630

        self.button_start_boundary = [(start_left, self.visual_img_height+button_top_offset),(start_left+button_width, self.visual_img_height+button_top_offset+button_height)]
        self.button_home_boundary  = [(home_left, self.visual_img_height+button_top_offset) ,(home_left+button_width , self.visual_img_height+button_top_offset+button_height)]
        self.button_abort_boundary = [(abort_left, self.visual_img_height+button_top_offset),(abort_left+button_width, self.visual_img_height+button_top_offset+button_height)]
        self.button_exit_boundary = [(app_exit_left, self.visual_img_height+button_top_offset),(app_exit_left+button_width, self.visual_img_height+button_top_offset+button_height)]

        self.start_button_pressed = False
        self.home_button_pressed = False
        self.abort_button_pressed = False
        self.exit_button_pressed = False

        self.user_command_msg = Int8()
        
        
        cv2.namedWindow("Task Assign")
        cv2.setMouseCallback("Task Assign", self.push_virtual_button)
        # cv2.createButton("Back",self.back,None,cv2.QT_PUSH_BUTTON,1)

        while True:
            # display the image and wait for a button-push

            if self.exit_button_pressed == True:
                break

            font = cv2.FONT_HERSHEY_SIMPLEX

            cv2.imshow("Task Assign", self.complete_image_with_text)
            key = cv2.waitKey(1) & 0xFF

            self.selection_img_init()
            cv2.rectangle( self.complete_image_with_text, self.button_start_boundary[0], self.button_start_boundary[1], (0,255,0), 5 )
            cv2.rectangle( self.complete_image_with_text, self.button_start_boundary[0], self.button_start_boundary[1], (100,255,100), -1 )
            UIstring = 'Start'
            cv2.putText(self.complete_image_with_text, UIstring, (self.button_start_boundary[0][0]+10, self.button_start_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)


            cv2.rectangle( self.complete_image_with_text, self.button_home_boundary[0], self.button_home_boundary[1], (0,255,0), 5 )
            cv2.rectangle( self.complete_image_with_text, self.button_home_boundary[0], self.button_home_boundary[1], (100,255,100), -1 )
            UIstring = 'Home'
            cv2.putText(self.complete_image_with_text, UIstring, (self.button_home_boundary[0][0]+10, self.button_home_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)


            cv2.rectangle( self.complete_image_with_text, self.button_abort_boundary[0], self.button_abort_boundary[1], (0,255,0), 5 )
            cv2.rectangle( self.complete_image_with_text, self.button_abort_boundary[0], self.button_abort_boundary[1], (100,255,100), -1 )
            UIstring = 'Abort'
            cv2.putText(self.complete_image_with_text, UIstring, (self.button_abort_boundary[0][0]+10, self.button_abort_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)


            cv2.rectangle( self.complete_image_with_text, self.button_exit_boundary[0], self.button_exit_boundary[1], (0,255,0), 5 )
            cv2.rectangle( self.complete_image_with_text, self.button_exit_boundary[0], self.button_exit_boundary[1], (100,255,100), -1 )
            UIstring = 'Exit'
            cv2.putText(self.complete_image_with_text, UIstring, (self.button_exit_boundary[0][0]+10, self.button_exit_boundary[0][1]+60 ), font, 1, (255, 0, 0), 2)


    # =============================================================
    # mouse event handle, for button 
    # =============================================================
    def push_virtual_button( self, event, x, y, flags, param):
        # if the left mouse button was clicked, record the starting (x, y) coordinates
        if event == cv2.EVENT_LBUTTONDOWN:
            # print('x,y: {} {}'.format(x,y))

            if self.button_start_boundary[0][0] < x < self.button_start_boundary[1][0] and self.button_start_boundary[0][1] < y < self.button_start_boundary[1][1] :
                self.start_button_pressed = True 
                print('\nstart pressed ')
                self.user_command_msg.data = 0
                self.user_pub.publish(self.user_command_msg)

                self.target_gps_pub.publish(self.targetlatlon)

            elif self.button_home_boundary[0][0] < x < self.button_home_boundary[1][0] and self.button_home_boundary[0][1] < y < self.button_home_boundary[1][1] :
                self.home_button_pressed = True 
                print('\nhome pressed ')
                self.user_command_msg.data = 1
                self.user_pub.publish(self.user_command_msg)

            elif self.button_abort_boundary[0][0] < x < self.button_abort_boundary[1][0] and self.button_abort_boundary[0][1] < y < self.button_abort_boundary[1][1] :
                self.abort_button_pressed = True 
                print('\nabort pressed ')
                self.user_command_msg.data = 2
                self.user_pub.publish(self.user_command_msg)

            elif self.button_exit_boundary[0][0] < x < self.button_exit_boundary[1][0] and self.button_exit_boundary[0][1] < y < self.button_exit_boundary[1][1] :
                self.exit_button_pressed = True 
                print('\nexit pressed ')


    # =============================================================
    # a tool for converting from pixel-location uv to lat-lon 
    # =============================================================
    def pix2latlon(self, uv):
        u = uv[0]
        v = uv[1]
        new_lon = self.current_lon_range[0] + (float(u)/self.visual_img_width) * (self.current_lon_range[1] - self.current_lon_range[0])
        new_lat = self.current_lat_range[1] + ( float(u) /self.visual_img_height ) * (self.current_lat_range[0] - self.current_lat_range[1]) 
        return (new_lat, new_lon)


    # =============================================================
    # a tool for computing which tile the input lat-lon locates at 
    # =============================================================
    def LatLonToTileIndex(self, lat, lon):
        lat_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lats) - lat )
        lon_diffs_from_all_tiles = np.abs( np.array( self.all_tile_center_lons) - lon ) 
        lat_index = np.argmin(lat_diffs_from_all_tiles)
        lon_index = np.argmin(lon_diffs_from_all_tiles)
        return [lon_index, lat_index]



    # ==========================================================
    # grab the map tiles and grid tiles following the inputs
    # ==========================================================
    def AssembleTiles(self, center_tile_x, center_tile_y, x_amount, y_amount ):
        if x_amount % 2 != 1 or y_amount % 2 != 1:
            print('tile assembly input size wrong ')

        print('assembling the tiles into map')

        check_each_image = False

        x_start = int( center_tile_x - (x_amount-1)/2 )
        y_start = int( center_tile_y + (y_amount-1)/2 )

        self.complete_image = np.zeros( ( x_amount * self.image_resolution , y_amount * self.image_resolution, 3) ).astype(np.uint8)
        sz = self.image_resolution  # this is only to use a shorter name for the image resolution in the below lines 

        # complete_grid  = np.zeros_like(self.complete_image)[:,:,0]

        for i in range(x_amount):
            x = x_start+i
            for j in range(y_amount):
                y = y_start-j 
                img_name = self.map_tiles_folder + '{}_{}.jpg'.format( x , y )

                onlyfiles = [f for f in listdir(self.map_tiles_folder) if isfile(join(self.map_tiles_folder, f))]
                if '{}_{}.jpg'.format( x , y ) not in onlyfiles:
                    self.request_map(x, y)

                one_tile = cv2.imread( img_name )
                # npy_name = 'maps/' + '{}_{}.npy'.format( x , y )
                # one_grid = np.load(npy_name)
                if check_each_image :
                    cv2.imshow( "map", cv2.resize( one_tile , (640,640) ) )
                    cv2.waitKey(30)
                    print( j*sz, (j+1)*sz, i*sz, (i+1)*sz )

                self.complete_image[ j*sz:(j+1)*sz, i*sz:(i+1)*sz ] = ( one_tile * 0.6 ).astype(np.uint8)
                # complete_grid[j*sz:(j+1)*sz, i*sz:(i+1)*sz] = one_grid

        lats = self.all_tile_corner_lats[y_start]
        lons = self.all_tile_corner_lons[x_start]

        print(lats, lons)

        self.top_left_latlon = [ max(lats) , min(lons) ]

        xhf , yhf = (x_amount-1)/2, (y_amount-1)/2

        top_left_tile_x_index = int(center_tile_x - xhf)
        top_left_tile_y_index = int(center_tile_y + yhf)

        bottom_right_tile_x_index = int(center_tile_x + xhf)
        bottom_right_tile_y_index = int(center_tile_y - yhf)

        self.current_lat_range = [ min( self.all_tile_corner_lats[bottom_right_tile_y_index] )  , max( self.all_tile_corner_lats[top_left_tile_y_index] )  ]
        self.current_lon_range = [ min( self.all_tile_corner_lons[top_left_tile_x_index] )  , max( self.all_tile_corner_lons[bottom_right_tile_x_index] )  ]

        print('range')
        print(self.current_lat_range)
        print(self.current_lon_range)

        self.lat_pix_ratio = ( self.current_lat_range[1] - self.current_lat_range[0] ) / (self.image_resolution * y_amount )
        self.lon_pix_ratio = ( self.current_lon_range[1] - self.current_lon_range[0] ) / (self.image_resolution * x_amount )

        return self.complete_image #, complete_grid

    # ==========================================================
    # Download the map tile with requested index 
    # ==========================================================
    def request_map(self, x_indx, y_indx): 
        print("Downloading tile:  x {}, y {} ".format(x_indx, y_indx))
        center_lon = self.all_tile_center_lons[x_indx]
        center_lat = self.all_tile_center_lats[y_indx]
        img_name = os.path.join("maps", "{}_{}.jpg".format(x_indx, y_indx))
        self.gen_map(center_lon, center_lat, self.zoom_level, self.image_resolution, img_name)

    # (lon, lat) is the center coordinate of the location
    def gen_map(self, lon, lat, zoom_level, img_resolution, img_name):
        lon_lat_command = "{},{},{},0,0".format(lon, lat, zoom_level)
        command = "https://api.mapbox.com/styles/v1/mapbox/satellite-streets-v11/static/{}/{}x{}?access_token={}".format(lon_lat_command, img_resolution, img_resolution, self.api_key)
        response = requests.get(command, stream=True)
        with open(img_name, 'wb') as out_file:
            shutil.copyfileobj(response.raw, out_file)




targetSelection()
