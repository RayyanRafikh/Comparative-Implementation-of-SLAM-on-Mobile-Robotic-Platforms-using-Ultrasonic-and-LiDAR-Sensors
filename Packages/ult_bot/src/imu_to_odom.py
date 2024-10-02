#!/usr/bin/python3

import rospy
import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TwistWithCovariance
from nav_msgs.msg import Odometry

import tf
from tf import transformations

class Odom():
    def __init__(self):

        # Initialzing a ros Node
        rospy.init_node("imu_to_odom")
        self.sub = rospy.Subscriber("/imu/data",Imu,self.callback)
        self.pub = rospy.Publisher("/odom",Odometry,queue_size=100)
        self.br = tf.TransformBroadcaster()

        self.X = 0
        self.Y = 0
        self.vel_x = 0
        self.prev_yaw = 0

        self.odom_msg = Odometry()
        self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.header.frame_id = "odom"

        self.flag = 1

        rospy.spin()
        pass

    def callback(self,data):

        t = 1/40
        self.odom_msg.header.stamp = data.header.stamp

        [roll,pitch,yaw] = self.euler_from_quaternion(data.orientation)

        if self.flag:
            self.yaw_bias = yaw
            pass
        self.flag = 0

        yaw = yaw - self.yaw_bias
        
        # Calculating the instantaneous distance and velocity by integrating
        inst_x = (self.vel_x * t) + (0.5 * data.linear_acceleration.x * (t**2))
        self.vel_x = self.vel_x + (data.linear_acceleration.x * t)

        self.X = self.X + (inst_x * math.cos(self.prev_yaw))
        self.Y = self.Y + (inst_x * math.sin(self.prev_yaw))

        # Updating the yaw
        self.prev_yaw = yaw

        # Populating the Pose field of the Odometry message
        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0
        self.odom_msg.pose.pose.orientation = data.orientation

        self.odom_msg.twist.twist.linear.x = self.vel_x
        self.odom_msg.twist.twist.linear.y = 0
        self.odom_msg.twist.twist.linear.z = 0
        self.odom_msg.twist.twist.angular = data.angular_velocity

        self.br.sendTransform((self.X, self.Y,0),(data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w),rospy.Time.now(),"base_link","odom")
        self.pub.publish(self.odom_msg)

        pass

    def euler_from_quaternion(self, q):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
if __name__ == '__main__':
    Odom()