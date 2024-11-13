#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import cos, sin, radians

class HuskyHighlevelController:
    def __init__(self):
        self.nodeHandle = rospy.init_node('back_home', anonymous=True)
        self.p_ang = 0.03
        self.p_vel = 0.8
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.LaserCallback)
        self.stop = False
    def LaserCallback(self, msg):
        if self.stop:
            return 0
        message = Twist()
        
        ranges = msg.ranges
        curvatures = []
        for i in range(len(ranges) - 1):
            if ranges[i] == 0 or ranges[i+1] == 0:
                curvatures.append(0)
            else:
                curvatures.append(abs(ranges[i] - ranges[i+1]))
        max_curvature_index = curvatures.index(max(curvatures)) + 1
        # if ranges[max_curvature_index] == 0:
        if (ranges[max_curvature_index] == 0):
            self.stop_robot(message)
        else:
            dist = ranges[max_curvature_index]
            if (max_curvature_index >= 180):
                angle = max_curvature_index - 360
            else:
                angle = max_curvature_index
            print("dist: ", dist)
            print("angle: ", angle)
            diff = angle
            message.angular.z = self.p_ang * diff
            vel = self.p_vel * (dist - 0.20)
            non_zero_elements = [x for x in msg.ranges if x != 0]
            if vel > 0.5:
                vel = 0.5
            elif (min(non_zero_elements) <= 0.15):
                # self.stop_robot(message)
                print(min(non_zero_elements))
                message.angular.z = 0
                vel = 0.0
                print("stop")
                self.stop = True
            # elif vel < 0.0:
            #     vel = 0.0        
            message.linear.x = vel
            self.vel_pub.publish(message)

    def stop_robot(self, message):
        message.linear.x = 0
        message.angular.z = 0
        print("Stop!")
        self.vel_pub.publish(message)

if __name__ == '__main__':
    controller = HuskyHighlevelController()
    rospy.spin()