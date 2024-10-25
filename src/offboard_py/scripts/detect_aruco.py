#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32

class ArucoDistanceCalculator:
    def __init__(self):
        rospy.init_node("aruco_distance_calculator")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/DIDALOS_vtol/usb_cam/image_raw", Image, self.image_callback)
        self.distance_x_pub = rospy.Publisher("aruco_center_x", Float32, queue_size=1)
        self.distance_y_pub = rospy.Publisher("aruco_center_y", Float32, queue_size=1)
        self.fx = rospy.get_param("~camera_fx", 277.191356)
        self.fy = rospy.get_param("~camera_fy", 277.191356)
        self.cx = rospy.get_param("~camera_cx", 160.25)
        self.cy = rospy.get_param("~camera_cy", 120.25)
        self.marker_size = rospy.get_param("~marker_size", 200.0)

        self.marker_window_name = "Marker"

        # Set up ArUco dictionary and parameters based on OpenCV version
        if cv2.__version__.startswith('3.') or cv2.__version__.startswith('4.'):
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:  # OpenCV 5.x and newer
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        if cv2.__version__.startswith('3.') or cv2.__version__.startswith('4.'):
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        else:  # OpenCV 5.x and newer
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            for i in range(len(ids)):
                distance_to_camera, distance_x, distance_y = self.get_distance_to_camera(corners[i])
                rospy.loginfo("Marker ID: %d, X error: %.2f pixel, Y error: %.2f pixel", ids[i][0], distance_x, distance_y)
                self.distance_x_pub.publish(Float32(distance_x))  
                self.distance_y_pub.publish(Float32(distance_y))  

                # Draw detected markers
                cv2.drawContours(cv_image, [corners[i].astype(int)], -1, (0, 255, 0), 2)
                cv2.arrowedLine(cv_image, (int(self.cx), int(self.cy)), 
                                (int(self.cx) + int(distance_x), int(self.cy) + int(distance_y)), 
                                (0, 0, 255), 2)
                
        else:
            rospy.logwarn("No Marker!!!")

        # Show image
        cv2.imshow(self.marker_window_name, cv_image)
        cv2.waitKey(1)

    def get_distance_to_camera(self, marker_corners):
        cx = int(np.mean(marker_corners[:, :, 0]))
        cy = int(np.mean(marker_corners[:, :, 1]))

        marker_pixel_size = max(marker_corners[:, :, 0].flatten()) - min(marker_corners[:, :, 0].flatten())
        distance = (self.marker_size * self.fx) / marker_pixel_size

        distance_x = cx - self.cx
        distance_y = cy - self.cy

        return distance, distance_x, distance_y

if __name__ == '__main__':
    try:
        node = ArucoDistanceCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass