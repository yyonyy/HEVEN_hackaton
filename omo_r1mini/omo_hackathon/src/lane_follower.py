#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from math import *
from collections import deque


def warpping(image):
    """
        차선을 BEV로 변환하는 함수
        
        Return
        1) _image : BEV result image
        2) minv : inverse matrix of BEV conversion matrix
    """
    source = np.float32([[140, 100], [0, 480], [500, 100], [640, 480]]) # 원본 좌표
    destination = np.float32([[0, 0], [0, 480], [480, 0], [480, 480]])  # warpping 할 좌표
    
    M = cv2.getPerspectiveTransform(source, destination)
    Minv = cv2.getPerspectiveTransform(destination, source)
    
    warp_image = cv2.warpPerspective(image, M, (480, 480), flags=cv2.INTER_LINEAR)

    return warp_image, Minv

def color_filter(image):
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    blue_lower = np.array([0, 51, 90])  #파란색 범위
    blue_upper = np.array([255, 204, 255])

    blue_mask = cv2.inRange(hls, blue_lower, blue_upper)
    masked = cv2.bitwise_and(image, image, mask = blue_mask)
    
    return masked

class lane_detect():
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber('/main_camera/image_raw/compressed', CompressedImage, self.camera_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.steer = 0

    
    def camera_callback(self, data):
        self.image = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.lane_detect()
        

    
    def high_level_detect(self, hough_img):

        nwindows = 10       # window 개수
        margin = 75         # window 가로 길이
        minpix = 30          # 차선 인식을 판정하는 최소 픽셀 수
       
        histogram = np.sum(hough_img[hough_img.shape[0]//2:,:],   axis=0)
    
        midx_current = np.argmax(histogram[:]) # window 시작점

        # 쌓을 window의 height 설정
        window_height = np.int32(hough_img.shape[0]/nwindows)
        
        # 0이 아닌 값을 저장한다.
        # nz[0]에는 index[row][col] 중에 row파트만 담겨있고 nz[1]에는 col이 담겨있다.
        nz = hough_img.nonzero()

        mid_lane_inds = []

        global x,y
        x,y = [],[]

        global out_img
        out_img = np.dstack((hough_img, hough_img, hough_img))*255

        mid_sum = 0

        total_loop = 0

        for window in range(nwindows-4):
            
            # bounding box 크기 설정
            win_yl = hough_img.shape[0] - (window+1)*window_height
            win_yh = hough_img.shape[0] - window*window_height

            win_xl = midx_current - margin
            win_xh = midx_current + margin

            # out image에 bounding box 시각화
            cv2.rectangle(out_img,(win_xl,win_yl),(win_xh, win_yh), (0,255,0), 2) 

            # 흰점의 픽셀들 중에 window안에 들어오는 픽셀인지 여부를 판단하여 
            # good_inds에 담는다.
            good_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)& (nz[1] >= win_xl)&(nz[1] < win_xh)).nonzero()[0]
            
            mid_lane_inds.append(good_inds)

            # nz[1]값들 중에 good_left_inds를 index로 삼는 nz[1]들의 평균을 구해서 leftx_current를 갱신한다.
            if len(good_inds) > minpix:
                midx_current = np.int32(np.mean(nz[1][good_inds]))

            #lx ly rx ry에 x,y좌표들의 중심점들을 담아둔다.
            x.append(midx_current)
            y.append((win_yl + win_yh)/2)

            # left_sum += leftx_current
            mid_sum += midx_current

            total_loop += 1

        mid_lane_inds = np.concatenate(mid_lane_inds)

        fit = np.polyfit(np.array(y[1:]),np.array(x[1:]),2)

        out_img[nz[0][mid_lane_inds], nz[1][mid_lane_inds]] = [255, 0, 0]

        mid_avg = mid_sum / total_loop

        return fit, mid_avg
    
    def lane_detect(self):
                
        cv2.namedWindow('Original') # 원본 이미지
        cv2.moveWindow('Original', 700, 0)
        cv2.imshow('Original', self.image)
        
        
        warpped_img, minv = warpping(self.image)    # BEV 전환
        # cv2.namedWindow('BEV')
        # cv2.moveWindow('BEV', 0, 0)
        # cv2.imshow('BEV', warpped_img)
        
        blurred_img = cv2.GaussianBlur(warpped_img, (7, 7), 5)  # blur 처리
        # cv2.namedWindow('Blurred')
        # cv2.moveWindow('Blurred', 350, 0)
        # cv2.imshow('Blurred', blurred_img)
        
        w_f_img = color_filter(blurred_img) # 파란색 검출
        cv2.rectangle(w_f_img, (0, 0), (480, 200), (0, 0, 0), -1) # ROI
        # cv2.namedWindow('Color filter')
        # cv2.moveWindow('Color filter', 0, 600)
        # cv2.imshow('Color filter', w_f_img)
        
        grayscale = cv2.cvtColor(w_f_img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(grayscale, 50, 255, cv2.THRESH_BINARY)
        
        canny_img = cv2.Canny(thresh, 10, 100)  # 경계 검출
        # cv2.namedWindow('Canny')
        # cv2.moveWindow('Canny', 350, 600)
        # cv2.imshow('Canny', canny_img)
        # cv2.namedWindow('thresh')
        # cv2.moveWindow('thresh', 700, 600)
        # cv2.imshow('thresh', thresh)
        
        lines = cv2.HoughLines(canny_img, 1, np.pi/180, 80, None, 0, 0) # 직선검출
        
        hough_img = np.zeros((480, 480))
        
        if lines is not None:
            for line in lines:
                
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                x1 = int (x0 + 1000*(-b))
                y1 = int ((y0) + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                
                slope = 90 - degrees(atan(b / a))
            
                if abs(slope) < 20:
                    cv2.line(hough_img, (x1, y1), (x2, y2), 0, 30)
                
                else:    
                    cv2.line(hough_img, (x1, y1), (x2, y2), 255, 8)
        
        # cv2.namedWindow('Hough')
        # cv2.moveWindow('Hough', 1400, 600)
        # cv2.imshow('Hough', hough_img)
        
        fit, avg = self.high_level_detect(hough_img)
        
        fit = np.polyfit(np.array(y),np.array(x),1)
        
        line = np.poly1d(fit)
        
        line_angle = degrees(atan(line[1])) #차선의 기울기


        cv2.namedWindow('Sliding Window')
        cv2.moveWindow('Sliding Window', 1400, 0)
        cv2.imshow("Sliding Window", out_img)
        cv2.waitKey(1)

        if fit[0]==0 and fit[1]==0:
            distance=0
        else:
            distance = -(np.polyval(fit,480) - 240) #차선과의 거리

        k= 0.001    #stanley 상수
        theta_err = radians(line_angle) #방향오차
        lat_err = distance * cos(line_angle)    #거리오차

        self.steer = theta_err + atan(k*lat_err)    #조향각계산

        # speed = Twist()
        # speed.linear.x = 0.1
        # speed.angular.z = theta_err + atan(k*lat_err)
        # self.pub.publish(speed)
        


if __name__ == "__main__":
    rospy.init_node('lane_detection_node', anonymous=False)
    if not rospy.is_shutdown():
        lane_detect()
        rospy.spin()
