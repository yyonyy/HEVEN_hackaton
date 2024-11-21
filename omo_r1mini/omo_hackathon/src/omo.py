#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from math import *
from database import Database
from geometry_msgs.msg import Twist
from lane_follower import lane_detect

class Omo():
    def __init__(self, hz = 10):
        self.rate = rospy.Rate(hz)

        self.db = Database()
        self.ld = lane_detect()
        self.db.init(self.rate)
        self.speed = 0 #초기 속도 설정

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def pub_cmd(self, speed, angle):    
        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = angle
        self.pub.publish(cmd)
    
    def main(self):
        
        # 1. 라이다 데이터를 받아오는 방법
        lidar_data = self.db.lidar_data

        # 2. 카메라 데이터를 받아오는 방법
        camera_data = self.db.image_data

        # 3. 차량 위치를 받아오는 방법
        pose_data = self.db.pose_data
        current_pose_x = pose_data.x
        current_pose_y = pose_data.y

        # 4. 현재 미션 받아오는 방법
        current_mission = self.db.current_mission 
        # print(current_mission)

        # 5. 차선주행
        lane_angle = self.ld.steer
        # print(lane_angle)

        # 6. 차량에 제어값을 보내는 방법
        # self.pub_cmd(self.speed,0)
        

if __name__ == "__main__":
    rospy.init_node("Omo")
    
    omo = Omo()
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            omo.main()
            omo.rate.sleep()
    except rospy.ROSInterruptException:
        pass