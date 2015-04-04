#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi

class Controller():
    def __init__(self):
        rospy.init_node('controller_node', anonymous=False)

        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=1)
        
        try:
            rospy.Subscriber("/robot0/laser_0", LaserScan, self.lidar_scan)
        except Exception as e:
            print(e)

        rate = 10
        
        r = rospy.Rate(rate)
        
        linear_speed = 0.2
        
        goal_distance = 1.0
        
        linear_duration = goal_distance / linear_speed
        
        angular_speed = 1.0
        
        goal_angle = pi
        
        angular_duration = goal_angle / angular_speed
     
        for i in range(2):
            move_cmd = Twist()
            
            move_cmd.linear.x = linear_speed
            
            ticks = int(linear_duration * rate)
            
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
            
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)
            
            move_cmd.angular.z = angular_speed

            ticks = int(goal_angle * rate)
            
            for t in range(ticks):           
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
            rospy.sleep(1)    
            
        self.cmd_vel.publish(Twist())
        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def lidar_scan(self,scan):
        print("print lidar:")
        print(scan)
        pass
 
if __name__ == '__main__':
    try:
        Controller()
    except:
        rospy.loginfo("node terminated.")

