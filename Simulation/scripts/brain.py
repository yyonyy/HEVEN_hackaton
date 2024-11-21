#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import time

from database import Database
from ackermann_msgs.msg import AckermannDrive
from parameter_list import Param
from goal import PARKING_SPOT, STOP_LINE, DELIV_PICKUP, DELIV_DROPOFF
from std_msgs.msg import Bool

param = Param()


class Brain():
    def __init__(self, db=Database, map_number=int):
        self.db = db
    
    def main(self):
        '''
        # 1. 라이다 데이터를 받아오는 방법
        lidar_data = self.db.lidar_data

        # 2. 차량 위치를 받아오는 방법
        pose_data = self.db.pose_data [x, y, yaw(degree)]

        # 3-1. 전역 경로 (Global Path)를 받아오는 방법
        global_path = self.db.global_path 
        [[x1, y1, yaw1(degree)], [x2, y2, yaw2], ...]
        # 3-2. 리스폰 직후 적절한 목표 global path index를 가져오는 방법
        respone = self.respone
        (True, False)
        respone_locations = self.db.respone_locations
        [[x1, y1], [x2, y2], ...]
        respone_idxes = self.db.respone_idxes
            바로 target하기 적절한 global path index
            [처음 지역, 횡단보도 종료 후, 장애물 종료 후, S자 주행 종료 후]

        # 4. 현재 미션을 확인하는 방법
        curr_mission = self.db.current_mission
        (정지선 : STOP_LINE)
        (장애물 : OBSTACLE)
        (S자 주행 : SCURVE)
        (주차 : PARKING_SPOT)

        # 각 미션에 필요한 데이터를 받아오는 방법
        # 5-1. 정지선에 대한 정보를 얻어오는 방법 
        stop_line = self.db.stop_line
        (None, STOP)
        remaining_time = self.db.traffic_remaining_time (남은 시간)
        # 5-2. 신호등 정보를 얻어오는 방법
        traffic_light = self.db.traffic_light
        (GREEN, RED)

        # 6-1. 주차 공간에 대한 정보를 얻어오는 방법
        parking_info = self.db.parking_list
            [x, y, yaw, num]
            (x : x 방향 상대 좌표 차이)
            (y : y 방향 상대 좌표 차이)
            (yaw : 주차 공간 방향 차이)
            (num : 주차 공간의 번호)
        # 6-2. 주차해야 하는 공간
        target_park = self.db.target_park

        
        # 최종적으로 차량의 각도와 속도 결정
        return angle, speed
        '''
        #=======================수정금지=======================
        lidar_data = self.db.lidar_data
        pose_data = self.db.pose_data
        global_path = self.db.global_path
        while len(global_path) == 0:
            global_path = self.db.global_path
        respone = self.db.respone
        respone_idxes = self.db.respone_idxes
        respone_locations = self.db.respone_locations
        curr_mission = self.db.current_mission
        stop_line = self.db.stop_line
        remaining_time = self.db.traffic_remaining_time
        traffic_light = self.db.traffic_light
        parking_info = self.db.parking_list
        target_park = self.db.target_park
        #=====================================================
        # 아래에 코드 작성
        """
        Determine the angle & speed
        angle(rad) : -1.8 ~ +1.8 (시계 방향-  반시계 방향+)
        speed(m/s) : 0 ~ 4       (음수면 후진)     
        """
        angle = 0
        speed = 0


        #=====================================================
        # Return angle & speed
        return angle, speed

def shutdown_handler(control_pub):
    rospy.loginfo("Shutting down... Publishing stop command.")
    stop_msg = AckermannDrive()  
    stop_msg.speed = 0
    stop_msg.steering_angle = 0
    for _ in range(5): 
        control_pub.publish(stop_msg)
        rospy.sleep(0.1)  

if __name__ == "__main__":
    db = Database(lidar=True)
    test_brain = Brain(db)
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('/drive', AckermannDrive, queue_size=1)
    timer_start_pub = rospy.Publisher('/timer_start', Bool, queue_size=1)

    # brain.py 종료시 차량을 정지시키는 코드. 수정X
    rospy.on_shutdown(lambda: shutdown_handler(control_pub))

    while not rospy.is_shutdown():
        car_angle, car_speed = test_brain.main()
        motor_msg = AckermannDrive()
        motor_msg.steering_angle = car_angle
        motor_msg.speed = car_speed
        motor_msg.steering_angle_velocity = param.car_angular_velocity
        motor_msg.acceleration = param.car_acceleration
        motor_msg.jerk = param.car_jerk
        control_pub.publish(motor_msg)
        timer_start_msg = Bool()
        timer_start_msg.data = True
        timer_start_pub.publish(timer_start_msg)
        rate.sleep()