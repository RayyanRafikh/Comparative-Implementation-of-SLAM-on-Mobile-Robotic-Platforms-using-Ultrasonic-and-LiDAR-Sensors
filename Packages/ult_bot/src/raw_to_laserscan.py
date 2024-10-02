#!/usr/bin/python3

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

def callback(msg):
    ls = LaserScan()

    ls.header.seq = seq
    ls.header.stamp = rospy.Time.now() 
    ls.header.frame_id = 'LaserScan_0'

    ls.angle_min = 0
    ls.angle_max = math.pi
    ls.angle_increment = math.radians(2048 / 360)
    
    ls.time_increment = 0.00005
    ls.scan_time = 0

    ls.range_min = 0
    ls.range_max = 0.5

    ls.ranges = msg.data / 100

    seq += 1

def listener():
    # Initialize listener
    rospy.Subscriber("arduino_array", LaserScan, callback)
    
    rospy.spin()


if __name__ == '__main__':
    seq = 0

    # Initialize publisher
    pub = rospy.Publisher('laser', LaserScan, queue_size=10)
    rospy.init_node('laser_handler', anonymous=True)

    listener();


