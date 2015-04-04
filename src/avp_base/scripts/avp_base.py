#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from math import pi,sin,cos
from time import sleep
import std_msgs
import serial
import simplejson as json
import tf

class AvpBase():
    def __init__(self,port='/dev/ttyACM0',baudrate=921600):
        rospy.init_node('avp_base_node', anonymous=False)

        rospy.on_shutdown(self.shutdown)
        
        self.imu = rospy.Publisher('/imu_data', Imu, queue_size=3)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=3)
        self.ser = serial.Serial(port, baudrate)
        br = tf.TransformBroadcaster()

        rate = 1
        
        r = rospy.Rate(rate)
        data=''
        while True:
            try:
                c=self.ser.read()
            except Exception,e:
                print("serial reading error")
                print(e)
                continue
            if c!='#':
                data+=c
            else:
                try:
                    res=json.loads(data)
                    imu_data = Imu()
                    header=std_msgs.msg.Header()
                    header.stamp=rospy.Time.now()
                    header.frame_id="/base_link"
                    imu_data.header=header
                    gyro=res['gyro']
                    acc=res['acc']
                    pos=res['pos']
                    #imu_data.orientation=tf.transformations.quaternion_from_euler(0, 0, gyro['hdg'])
                    quaternion = Quaternion()
                    quaternion.x = 0.0 
                    quaternion.y = 0.0
                    quaternion.z = sin(gyro['hdg'] / 2.0)
                    quaternion.w = cos(gyro['hdg'] / 2.0)
                    imu_data.orientation=quaternion
    
                    imu_data.angular_velocity.z=gyro['yv']
                    imu_data.linear_acceleration.x=acc['x']
                    imu_data.linear_acceleration.y=acc['y']
                    imu_data.linear_acceleration.z=acc['z']
                    self.imu.publish(imu_data)

                    odom_data=Odometry()
                    odom_data.header.stamp=rospy.Time.now()
                    odom_data.header.frame_id = "/odom";
                    odom_data.child_frame_id = "/base_link";
                    odom_data.pose.pose.position.x = pos['x'];
                    odom_data.pose.pose.position.y = pos['y'];
                    odom_data.pose.pose.position.z = 0.0;
                    odom_data.pose.pose.orientation = imu_data.orientation

                    odom_data.twist.twist.linear.x = acc['x'];
                    odom_data.twist.twist.linear.y = acc['y'];
                    odom_data.twist.twist.angular.z = gyro['yv'];
                    self.odom.publish(odom_data)
                    br.sendTransform((pos['x'], pos['y'], 0.0),
                         imu_data.orientation,
                         rospy.Time.now(),
                         "/odom",
                         "/base_link")

                    br.sendTransform((0.2, 0.0, 0.0),
                         (0.0,0.0,0.0,1.0),
                         rospy.Time.now(),
                         "/base_link",
                         "/base_laser")
                    print(res)
                except Exception,e:
                    rospy.loginfo('error in sensor parsing')
                    rospy.loginfo(e)
                    print(data)
                r.sleep()
                data=''
            #r.sleep()
    def shutdown(self):
        rospy.loginfo("shutting down the robot...")
 
if __name__ == '__main__':
    try:
        AvpBase()
    except Exception,e:
        print(e)
        rospy.loginfo("avp-base node terminated.")

