#!/usr/bin/python3

import rospy
import serial
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistWithCovariance
import tf
from tf import transformations

class IMU_Node():

    def __init__(self):
        
        # Initializing the ROS node and the publisher node
        rospy.init_node("IMU_Node")
        pub = rospy.Publisher("/odom", Odometry, queue_size=100)
        self.br = tf.TransformBroadcaster()

        # Getting the port from ros_param
        port = rospy.get_param("/imu_port","/dev/ttyUSB0")

        # Initializing the message variables
        imu_msg = Imu()
        odom_msg = Odometry()

        # Assigning the frame IDs
        imu_msg.header.frame_id = "base_link"
        

        self.vel_x = 0
        self.prev_yaw = 0
        self.X = 0
        self.Y = 0

        self.wind_acc_x = [0,0,0,0,0]
        self.wind_acc_z = [0,0,0,0,0]
        
        flag = 1

        # Reading from the serial port
        try:
            ser = serial.Serial(port, 115200, timeout=1)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            exit()
        
        while(not rospy.is_shutdown()):
            
            raw_data = ser.readline().decode()
            raw_data = raw_data.split(',')

            t = rospy.Time.now()
            imu_msg.header.stamp = t
            odom_msg.header.stamp = t

            # Populating Imu variable fields
            try:
                imu_x = float(raw_data[7][:4])
                imu_msg.linear_acceleration.y = float(raw_data[8])
                imu_z = float(raw_data[9][:4])

                imu_msg.angular_velocity.x = float(raw_data[10])
                imu_msg.angular_velocity.y = float(raw_data[11])
                imu_msg.angular_velocity.z = float(raw_data[12][:-5])

                yaw = float(raw_data[1])
                pitch = float(raw_data[2])

            except Exception as e:
                print("Try block failed!:")
                print(e)
                continue

            # Getting the bias to eliminate zero-error
            if(flag):
                accel_x_bias = imu_x
                yaw_bias = yaw
                # g = ((imu_msg.linear_acceleration.x ** 2) + (imu_msg.linear_acceleration.y ** 2) + (imu_msg.linear_acceleration.z ** 2)) ** 0.5
                pass
            
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.z = self.window_filt(imu_x,imu_z)

            # Correcting for zero-error
            imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.x - (imu_msg.linear_acceleration.z * math.sin(pitch * math.pi / 180)) #- accel_x_bias  # The negative of pitch and z axis cancel out
            yaw = -(yaw - yaw_bias)

            odom_msg = self.get_odom(imu_msg.linear_acceleration.x,imu_msg.angular_velocity.z,yaw,t)
            odom_msg.child_frame_id = "base_link"
            odom_msg.header.frame_id = "map"

            pub.publish(odom_msg)

            flag = 0
            pass

    def window_filt(self,acc_x, acc_z):
        
        sum_x = 0
        sum_z = 0
        # self.wind_acc_x[0] = self.wind_acc_x[1]
        # self.wind_acc_x[1] = self.wind_acc_x[2]
        # self.wind_acc_x[2] = acc_x
        n = len(self.wind_acc_x)
        for i in range(n - 1):
            self.wind_acc_x[i] = self.wind_acc_x[i+1]
            self.wind_acc_z[i] = self.wind_acc_z[i+1]

            sum_x = sum_x + self.wind_acc_x[i]
            sum_z = sum_z + self.wind_acc_z[i]
            pass
        
        self.wind_acc_x[n-1] = acc_x
        self.wind_acc_z[n-1] = acc_z

        sum_x = sum_x + acc_x
        sum_z = sum_z + acc_z

        mean_x = sum_x / n

        # self.wind_acc_z[0] = self.wind_acc_z[1]
        # self.wind_acc_z[1] = self.wind_acc_z[2]
        # self.wind_acc_z[2] = acc_z

        mean_z = sum_z / n

        return mean_x,mean_z

    
    def get_odom(self,accel_x,gyro_z,yaw,t_ros):
            
        t = 1/40
        yaw_rad = yaw * math.pi / 180
        
        quat = Quaternion()
        twist_var = TwistWithCovariance()
        odom = Odometry()
        
        
        # Getting the instantaneous distance travelled and current velocity
        inst_x = (self.vel_x * t) + (0.5 * accel_x * (t**2))
        self.vel_x = self.vel_x + (accel_x * t)

        # print("Dist: ",inst_x)
        # print("vel: ",self.vel_x)
        
        # Getting position w.r.t odom frame
        self.X = self.X + inst_x * math.cos(self.prev_yaw)
        self.Y = self.Y + inst_x * math.sin(self.prev_yaw)

        # self.X = inst_x * math.cos(self.prev_yaw)
        # self.Y = inst_x * math.sin(self.prev_yaw)
        
        # Updating the yaw
        self.prev_yaw = yaw_rad
        
        # Convering Euler angles to Quaternion
        quat.w = math.cos((yaw_rad)/2)
        quat.x = 0
        quat.y = 0
        quat.z = math.sin((yaw_rad)/2)
        
        odom.pose.pose.position.x = self.X
        odom.pose.pose.position.y = self.Y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quat
        
        # twist_var.twist.linear.x = self.vel_x # * math.cos(self.prev_yaw)
        # twist_var.twist.linear.y = 0 # self.vel_x * math.sin(self.prev_yaw)
        # twist_var.twist.linear.z = 0
        
        # twist_var.twist.angular.x = 0
        # twist_var.twist.angular.y = 0
        # twist_var.twist.angular.z = gyro_z
        
        odom.twist.twist.linear.x = self.vel_x
        odom.twist.twist.linear.y = 0
        odom.twist.twist.linear.z = 0
        
        odom.twist.twist.angular.x = 0
        odom.twist.twist.angular.y = 0
        odom.twist.twist.angular.z = gyro_z

        self.br.sendTransform((self.X,self.Y,0),(quat.x,quat.y,quat.z,quat.w),t_ros,"base_link","map")
        
        return odom

if __name__ == '__main__':
    IMU_Node()
    pass