#!/usr/bin/python3

import rospy
import serial

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

class IMU_Node():

    def __init__(self):
        
        # Initialzing the node and the publishers
        rospy.init_node("IMU_reader")
        pub_im = rospy.Publisher("imu/data_raw",Imu, queue_size=100)
        pub_mag = rospy.Publisher("imu/mag",MagneticField, queue_size=100)

        # Getting the port
        port = rospy.get_param("/imu_port","/dev/ttyUSB0")

        imu_msg = Imu()
        mag_msg = MagneticField()

        imu_msg.header.frame_id = "base_link"
        mag_msg.header.frame_id = "base_link"

        try:
            ser = serial.Serial(port, 115200, timeout=1)
        except rospy.ServiceException as e:
            print("Error!: %s"%e)
            exit()

        while(not rospy.is_shutdown()):
            
            raw_data = ser.readline().decode()
            raw_data = raw_data.split(',')

            try:
                imu_msg.linear_acceleration.x = float(raw_data[7])
                imu_msg.linear_acceleration.y = float(raw_data[8])
                imu_msg.linear_acceleration.z = float(raw_data[9])

                imu_msg.angular_velocity.x = float(raw_data[10])
                imu_msg.angular_velocity.y = float(raw_data[11])
                imu_msg.angular_velocity.z = float(raw_data[12][:-5])


                mag_msg.magnetic_field.x = float(raw_data[4])
                mag_msg.magnetic_field.y = float(raw_data[5])
                mag_msg.magnetic_field.z = float(raw_data[6])

            except Exception as e:
                print("Error! %s"%e)
                continue

            t = rospy.Time.now()
            imu_msg.header.stamp = t
            mag_msg.header.stamp = t

            pub_im.publish(imu_msg)
            pub_mag.publish(mag_msg)

        pass

if __name__ == '__main__':
    IMU_Node()