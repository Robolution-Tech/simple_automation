#!/usr/bin/env python
import rospy
import serial
import os
import json
import time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


os.system('sudo chmod 666 /dev/ttyACM0') #TODO: add to system config



GPS_port_available = False
while(GPS_port_available == False and not rospy.is_shutdown()):
    try:
        # Create an IMU instance
        ser = serial.Serial(port='/dev/ttyACM0')
        GPS_port_available = True
    except:
        print("GPS fails to initialize, check port")
        time.sleep(1)



ser.flushInput()

print("connected to: " + ser.portstr)
# count=1
rospy.init_node('custom_ublox_gps_node')

equipment_gps_location_topic = rospy.get_param(
            '/robo_param/topic_names/equipment_gps_location_topic', "equipment_gps")

pub = rospy.Publisher(equipment_gps_location_topic, NavSatFix, queue_size=10)
pub_state = rospy.Publisher('ublox_gps_ready', Bool, queue_size=10)


#setPAth = 'set' + '4'
#print('saving to ', setPAth)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    # print('loop, {}'.format(time.time()) )
    r.sleep(1) #1hz
    try:
        ser_bytes = ser.readline()  # $GNGLL,5318.72869,N,11334.84164,W,191706.00,A,A*6B
        decoded_bytes = ser_bytes.decode("utf-8")
        
        # if GNGGA
        #print(decoded_bytes)
        if decoded_bytes[0:6] == '$GNGGA':
            # print(decoded_bytes)
            gps_state = Bool()
            if decoded_bytes[6:10] != ',,,,':
                #raw = decoded_bytes[7:33].split(',')
                raw = decoded_bytes.split(',')
                #print('-------------',raw[2],raw[4])
                
                navfix = NavSatFix()
                navfix.header.stamp = rospy.get_rostime()
                navfix.header.frame_id = "gps_link"
                navfix.status.status = int(raw[6])
                navfix.status.service = 1
                hdop = float(raw[8])
                
                raw_lat_string = raw[2]
                raw_lon_string = raw[4]
                raw_alt_string = raw[9]
                #print(raw_lat_string,raw_lon_string)
                
                lat_deg = int(raw_lat_string[0:2])    
                lat_min = float(raw_lat_string[2:])/60.0
                lat_dec = lat_deg + lat_min

                lon_deg = int(raw_lon_string[0:3])
                lon_min = float(raw_lon_string[3:])/60.0
                lon_dec = lon_deg + lon_min
                lon_dec *= -1.0
                
                print('%.6f , %.6f , %.1f'%(lat_dec,  lon_dec, float(raw_alt_string) ))

                navfix.latitude  = lat_dec
                navfix.longitude = lon_dec
                navfix.altitude = float(raw_alt_string)
                navfix.position_covariance[0] = hdop ** 2
                navfix.position_covariance[4] = hdop ** 2
                navfix.position_covariance[0] = ( 2*hdop ) ** 2
                navfix.position_covariance_type = 1
                
                pub.publish(navfix)
                
                gps_state.data = True
                pub_state.publish(gps_state)
                
                # count = count+1
                
                # time.sleep(1.0)
                
            else:
                print('gps initilizing')
                
                gps_state.data = False
                pub_state.publish(gps_state)
                time.sleep(2)
                

    except Exception as e: 
        print(e)
        print("Keyboard Interrupt")
        break
        

    
    


ser.close()



