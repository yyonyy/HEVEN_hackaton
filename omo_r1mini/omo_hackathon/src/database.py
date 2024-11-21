#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import LaserScan, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import math

class Database():
    def __init__(self, lidar=True):
        # init node
        # rospy.init_node('sensor_node')

        # sensor subscriber
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        self.bridge = CvBridge()
        rospy.Subscriber('/main_camera/image_raw/compressed', CompressedImage, self.camera_callback, queue_size=1)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Data
        self.lidar_data = None
        self.pose_data = None
        self.image_data = None
        self.current_mission = "stop_line"
        self.current_mission_index = 0

        self.mission_sequence = [
            (1.2737924388391881, -0.5185944642433705, "obstacle"),
            (0.042972605238863835, -1.5254457326403026, "curve"),
            (1.4459617040153936, -1.2019157410520829, "left_turn"),
            (0.7610406963663024, -0.9045210467863436, "parking")
        ]

        self.mission_tolerance = 0.25

    def camera_callback(self, data=CompressedImage):
        self.image_data = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")

    def lidar_callback(self, data=LaserScan):
        self.lidar_data = data.ranges

    def pose_callback(self, data=PoseWithCovarianceStamped):
        self.pose_data =data.pose.pose.position

        if self.pose_data is not None and self.current_mission_index < len(self.mission_sequence):
            current_x = self.pose_data.x
            current_y = self.pose_data.y

            # Get the next mission start point
            mission_x, mission_y, mission_name = self.mission_sequence[self.current_mission_index]
            
            # Calculate distance to the next mission start point
            distance = math.sqrt((current_x - mission_x) ** 2 + (current_y - mission_y) ** 2)

            # Transition to the next mission if within tolerance
            if distance <= self.mission_tolerance:
                self.current_mission = mission_name
                self.current_mission_index += 1
                rospy.loginfo(f"Transitioned to mission: {self.current_mission}")



    def init(self, rate):
        '''
        wait for initial callbacks from all sensors
        '''
        while self.image_data is None:
            rate.sleep()
            if rospy.is_shutdown:
                break
        print("cam_ready")
        
        while self.lidar_data is None:
            rate.sleep()
            if rospy.is_shutdown:
                break
        print("scan_ready")

        while self.pose_data is None:
            rate.sleep()
            if rospy.is_shutdown:
                break
        print("pose_ready")
