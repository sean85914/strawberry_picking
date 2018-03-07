#!/usr/bin/env python
import serial
import time
import rospy
import tf
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
marker_id = 0

def rv2NavSatFix(info):
    fix = NavSatFix()
    latitude_DDMM = float(info[3])
    latitude_100 = latitude_DDMM / 100
    latitude_int = int(latitude_100)
    latitude_dec = latitude_100 - latitude_int
    latitude_deg = latitude_dec * 10 / 6
    fix.latitude = latitude_int + latitude_deg
    #print fix.latitude
    if info[4] == 'S':
        fix.latitude = -fix.latitude
    longitude_DDMM = float(info[5])
    longitude_100 = longitude_DDMM / 100
    longitude_int = int(longitude_100)
    longitude_dec = longitude_100 - longitude_int
    longitude_deg = longitude_dec *10 / 6
    fix.longitude = longitude_int + longitude_deg
    if info[6] == 'W':
        fix.longitude = -fix.longitude
    '''fix.status.status = NavSatStatus.STATUS_FIX
    fix.status.service = NavSatStatus.SERVICE_GPS
    fix.altitude = float('NaN')
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN'''
    fix.header.frame_id = 'gps'
    fix.header.stamp = rospy.get_rostime()
    gpspub.publish(fix)
    position = Marker()
    position.header.frame_id = 'gps'
    position.header.stamp = rospy.Time.now()
    position.lifetime = rospy.Duration()
    position.action = Marker.ADD
    #position.pose.position.x = fix.latitude
    #position.pose.position.y = fix.longitude
    position.pose.orientation.w = 1.0
    global marker_id 
    position.id = marker_id
    marker_id += 1
    position.type = Marker.POINTS
    position.scale.x = 0.2
    position.scale.y = 0.2
    position.color.g = 1.0
    position.color.a = 1.0
    p = Point()
    p.x = fix.latitude / 100
    p.y = fix.longitude / 100
    position.points.append(p)
    marker_pub.publish(position)
def readlineCR(port):
    rv = ""
    if port.read()=='$':
        while True:
            ch = port.read()
            rv += ch
            if ch=='\r' or ch=='':
                info = rv.split(',')
                if info[0] == 'GPRMC':
                    rv2NavSatFix(info)
                    return info
                else:
                    return None
    return None

if __name__ == '__main__':
    #init publisher
    rospy.init_node('gps_info_pub')
    gpspub = rospy.Publisher('/fix', NavSatFix,queue_size=1)
    marker_pub = rospy.Publisher('/gps/marker', Marker, queue_size=1)
    port = serial.Serial("/dev/ttyUSB0", baudrate=4800, timeout=1)
    #rate = rospy.Rate(10) # 10hz
    try:
        while not rospy.is_shutdown():
            rcv = readlineCR(port)
            if rcv != None:
                print rcv
            #rate.sleep()
    except rospy.ROSInterruptException:
        pass
