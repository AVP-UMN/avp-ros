#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi

class LandmarkDetector():
    def __init__(self):
        rospy.init_node('landmark_detector', anonymous=False)

        rospy.on_shutdown(self.shutdown)
                
        try:
            rospy.Subscriber("/scan", LaserScan, self.lidar_scan)
        except Exception as e:
            print(e)

        rospy.spin()
            
        #self.cmd_vel.publish(Twist())
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        #self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def lidar_scan(self,scan):
        print("print lidar:")
        print(scan)
        pass
 
if __name__ == '__main__':
    try:
        LandmarkDetector()
    except:
        rospy.loginfo("node terminated.")

