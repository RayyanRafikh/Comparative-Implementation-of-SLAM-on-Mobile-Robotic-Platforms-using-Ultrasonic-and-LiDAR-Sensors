#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray  

def distance_angle_callback(data):
    # Assuming DistanceAngleMsg has 'distance' and 'angle' fields
    distances = []
    for i in range(len(data.data)):
    	if(data.data[i] > 30):
    	    distances.append(float('inf'))
    	else: 
    	    distances.append(float(data.data[i]/100))

    # Create a LaserScan message
    laserscan_msg = LaserScan()
    laserscan_msg.header.stamp = rospy.Time.now()
    laserscan_msg.header.frame_id = "frame_id"  # Replace with the appropriate frame_id

    # Populate LaserScan message fields
    laserscan_msg.angle_min = 0    
    laserscan_msg.angle_max = 180 * (np.pi / 180)
    laserscan_msg.angle_increment = 2 * (np.pi / 180)
    laserscan_msg.time_increment = 0.1  # Set appropriate time increment
    laserscan_msg.scan_time = 9.0  # Set appropriate scan time
    laserscan_msg.range_min = 0.05  # Set appropriate minimum range
    laserscan_msg.range_max = 1 # Replace with the maximum range

    # Assuming you have a list of distances named 'distances'
    laserscan_msg.ranges = distances

    # Publish the LaserScan message to a new topic
    laserscan_publisher.publish(laserscan_msg)

if __name__ == '__main__':
    rospy.init_node('distance_angle_to_laserscan')

    # Replace 'your_topic' with the actual topic where distance and angle data is published
    rospy.Subscriber('laser_data', Int16MultiArray, distance_angle_callback)

    # Replace 'your_laserscan_topic' with the desired topic for the LaserScan message
    laserscan_publisher = rospy.Publisher('laserscan', LaserScan, queue_size=10)

    # Define other parameters like 'num_of_readings' and 'max_range' appropriately

    rospy.spin()
