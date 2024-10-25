#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest, SetMavFrame
from std_msgs.msg import Float32
from simple_pid import PID
import numpy as np
import csv
import os

# 가속도 고려 x
class KalmanFilter:
    def __init__(self, dt=0.1):
        self.dt = dt

        self.x = np.zeros(4)

        self.F = np.array([[1, 0, self.dt, 0],
                           [0, 1, 0, self.dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])

        self.Q = np.eye(4) * 0.01

        self.R = np.eye(2) * 0.1

        self.P = np.eye(4)

    def predict(self):
        self.x = np.dot(self.F, self.x)
        
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))
        
        self.x = self.x + np.dot(K, y)
        
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

    def get_state(self):
        return self.x[0:2]


# 가속도 고려 O -> 가속도는 시스템 잡음으로...
class KalmanFilter_acc:
    def __init__(self, dt=0.1):
        self.dt = dt

        # 상태 벡터: [x, y, vx, vy, ax, ay]
        self.x = np.zeros(6)

        # 상태 전이 행렬(F): 위치, 속도, 가속도를 고려
        self.F = np.array([[1, 0, self.dt, 0, 0.5 * self.dt ** 2, 0],
                           [0, 1, 0, self.dt, 0, 0.5 * self.dt ** 2],
                           [0, 0, 1, 0, self.dt, 0],
                           [0, 0, 0, 1, 0, self.dt],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])

        # 측정 행렬(H): 측정값은 위치(x, y)만 사용
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0]])

        # 시스템 잡음 공분산 행렬(Q): 가속도의 불확실성을 반영
        self.Q = np.eye(2) * 0.01

        # 측정 잡음 공분산 행렬(R): 측정 불확실성 반영
        self.R = np.eye(2) * 0.1

        # 초기 공분산 행렬(P)
        self.P = np.eye(6)

    def predict(self):
        # 상태 예측
        self.x = np.dot(self.F, self.x)

        # 공분산 예측 (Gamma 행렬은 가속도 관련 시스템 잡음을 반영) 
        Gamma = np.array([[0.5 * self.dt ** 2, 0],
                          [0, 0.5 * self.dt ** 2],
                          [self.dt, 0],
                          [0, self.dt],
                          [1, 0],
                          [0, 1]])

        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + np.dot(Gamma, np.dot(self.Q, Gamma.T))

    def update(self, z):
        # 측정 혁신
        y = z - np.dot(self.H, self.x)

        # 혁신 공분산
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # 칼만 이득
        K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(S)))

        # 상태 갱신
        self.x = self.x + np.dot(K, y)

        # 공분산 갱신
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

    def get_state(self):
        # 상태 벡터에서 위치 반환
        return self.x[0:2]



class OffboardController:
    def __init__(self, method_name):
        rospy.init_node("winter")
        
        # State
        self.current_state = State()
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        
        # Position publishing
        self.local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # Velocity publishing
        self.vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

        # Altitude
        self.rel_alt = rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.alt_callback)

        # Arming
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

        # Set mode
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.rate = rospy.Rate(50)

        self.aruco_x = rospy.Subscriber("aruco_center_x", Float32, self.aruco_x_callback)
        self.aruco_y = rospy.Subscriber("aruco_center_y", Float32, self.aruco_y_callback)

        self.yolo_x = rospy.Subscriber("yolo_center_x", Float32, self.yolo_x_callback)
        self.yolo_y = rospy.Subscriber("yolo_center_y", Float32, self.yolo_y_callback)


        self.pids = {'x': PID(0.5, 0.0, 0.2, setpoint=0), 'y': PID(0.5, 0.0, 0.2, setpoint=0)}

        # Generate different CSV file names based on the landing method
        self.csv_file_path = os.path.expanduser(f"~/data/landing_data_{method_name}.csv")
        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write the CSV header including position data
        self.csv_writer.writerow(['cp_x', 'cp_y', 'altitude', 'x', 'y', 'z'])

        # Subscribe to the local position to get x, y, z coordinates
        self.local_position_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

        # Store the current position data
        self.current_position = None
    
    def state_cb(self, msg):
        self.current_state = msg

    def alt_callback(self, msg):
        self.rel_alt.data = msg.data

    def pose_callback(self, msg):
        self.curr_pose = msg

    def aruco_x_callback(self, msg):
        self.aruco_x_error = msg.data

    def aruco_y_callback(self, msg):
        self.aruco_y_error = msg.data

    def yolo_x_callback(self, msg):
        self.yolo_x_error = msg.data

    def yolo_y_callback(self, msg):
        self.yolo_y_error = msg.data

    def pose_callback(self, msg):
        # Update current position when new data is received
        self.current_position = msg.pose.position

    def update_csv(self, cp_x, cp_y, altitude):
        if self.current_position is not None:
            # Log cp_x, cp_y, altitude, and position (x, y, z)
            self.csv_writer.writerow([cp_x, cp_y, altitude,
                                      self.current_position.x, self.current_position.y, self.current_position.z])

    def close_csv(self):
        self.csv_file.close()

    def set_mav_frame(self, frame_id):
        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            mav_frame_service = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMavFrame)
            response = mav_frame_service(frame_id)
            if response.success:
                print("MAV frame set successfully to", frame_id)
            else:
                print("Failed to set MAV frame")
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def takeoff(self, alt):
        while not rospy.is_shutdown() and not self.current_state.connected:
            self.rate.sleep()

        altitude = PoseStamped()
        altitude.pose.position.x = 0
        altitude.pose.position.y = 0
        altitude.pose.position.z = alt

        for i in range(100):
            if rospy.is_shutdown():
                break

            self.local_pos_pub.publish(altitude)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while not rospy.is_shutdown():
            current_pose = rospy.wait_for_message(
                "/mavros/local_position/pose", PoseStamped
            )

            dist_to_waypoint = (
                (current_pose.pose.position.x - altitude.pose.position.x) ** 2
                + (current_pose.pose.position.y - altitude.pose.position.y) ** 2
                + (current_pose.pose.position.z - altitude.pose.position.z) ** 2
            ) ** 0.5

            if dist_to_waypoint < 0.25:
                break

            if self.current_state.mode != "OFFBOARD" and (
                rospy.Time.now() - last_req
            ) > rospy.Duration(5.0):
                if self.set_mode_client.call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if not self.current_state.armed and (
                    rospy.Time.now() - last_req
                ) > rospy.Duration(5.0):
                    if self.arming_client.call(arm_cmd).success:
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            self.local_pos_pub.publish(altitude)

            self.rate.sleep()

    def land(self):
        rospy.loginfo("Setting AUTO.LAND mode...")
        land_set_mode = SetModeRequest()
        land_set_mode.custom_mode = "AUTO.LAND"
        try:
            if self.set_mode_client.call(land_set_mode).mode_sent:
                rospy.loginfo("AUTO.LAND mode enabled")
            else:
                rospy.logerr("Failed to set AUTO.LAND mode")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def aruco_land(self):
        while not rospy.is_shutdown():
            vel_cmd = Twist()
            k = 0.01

            cp_x = self.pids['x'](self.aruco_x_error)
            cp_y = self.pids['y'](self.aruco_y_error)

            # Log and save the data
            self.update_csv(cp_x, cp_y, self.rel_alt.data)
            
            print('aruco', cp_x, cp_y)
            print('alttitude = ', self.rel_alt.data)
            vel_cmd.linear.x = k * cp_y
            vel_cmd.linear.y = k * cp_x
            vel_cmd.linear.z = -0.3
            self.vel_pub.publish(vel_cmd)
            
            if abs(self.rel_alt.data - 1) < 0.01:
                vel_cmd.linear.x = 0.0
                vel_cmd.linear.y = 0.0
                vel_cmd.linear.z = 0.0
                self.vel_pub.publish(vel_cmd)

                rospy.loginfo("Setting AUTO.LAND mode...")
                land_set_mode = SetModeRequest()
                land_set_mode.custom_mode = "AUTO.LAND"
                try:
                    if self.set_mode_client.call(land_set_mode).mode_sent:
                        rospy.loginfo("AUTO.LAND mode enabled")
                    else:
                        rospy.logerr("Failed to set AUTO.LAND mode")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")

            self.rate.sleep()


    def yolo_land(self):
        while not rospy.is_shutdown():
            vel_cmd = Twist()
            k = 0.01

            cp_x = self.pids['x'](self.yolo_x_error)
            cp_y = self.pids['y'](self.yolo_y_error)

            # Log and save the data
            self.update_csv(cp_x, cp_y, self.rel_alt.data)

            print('yolo = ',cp_x, cp_y)
            print('alttitude = ', self.rel_alt.data)
            vel_cmd.linear.x = k * cp_y
            vel_cmd.linear.y = k * cp_x
            vel_cmd.linear.z = -0.3
            self.vel_pub.publish(vel_cmd)
            
            if abs(self.rel_alt.data - 1) < 0.01:
                vel_cmd.linear.x = 0.0
                vel_cmd.linear.y = 0.0
                vel_cmd.linear.z = 0.0
                self.vel_pub.publish(vel_cmd)

                rospy.loginfo("Setting AUTO.LAND mode...")
                land_set_mode = SetModeRequest()
                land_set_mode.custom_mode = "AUTO.LAND"
                try:
                    if self.set_mode_client.call(land_set_mode).mode_sent:
                        rospy.loginfo("AUTO.LAND mode enabled")
                    else:
                        rospy.logerr("Failed to set AUTO.LAND mode")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")

            self.rate.sleep()

    def kalman_land(self):
        # Kalman filter for position estimation
        kalman_filter = KalmanFilter(dt=0.1)

        while not rospy.is_shutdown():
            vel_cmd = Twist()
            k = 0.01

            # Get the Aruco and Yolo data (measurements)
            z_aruco = np.array([self.aruco_x_error, self.aruco_y_error])
            z_yolo = np.array([self.yolo_x_error, self.yolo_y_error])

            # Kalman filter prediction step
            kalman_filter.predict()

            # Kalman filter update with both Aruco and Yolo data
            kalman_filter.update(z_aruco)
            kalman_filter.update(z_yolo)

            # Get the estimated state (center position)
            estimated_position = kalman_filter.get_state()

            # Control based on the Kalman filter output
            print('Estimated Position = ', estimated_position)
            print('Altitude = ', self.rel_alt.data)

            cp_x = self.pids['x'](estimated_position[0])
            cp_y = self.pids['y'](estimated_position[1])

            # Log and save the data
            self.update_csv(cp_x, cp_y, self.rel_alt.data)

            vel_cmd.linear.x = k * cp_y
            vel_cmd.linear.y = k * cp_x
            vel_cmd.linear.z = -0.3
            self.vel_pub.publish(vel_cmd)

            if abs(self.rel_alt.data - 1) < 0.01:
                vel_cmd.linear.x = 0.0
                vel_cmd.linear.y = 0.0
                vel_cmd.linear.z = 0.0
                self.vel_pub.publish(vel_cmd)

                rospy.loginfo("Setting AUTO.LAND mode...")
                land_set_mode = SetModeRequest()
                land_set_mode.custom_mode = "AUTO.LAND"
                try:
                    if self.set_mode_client.call(land_set_mode).mode_sent:
                        rospy.loginfo("AUTO.LAND mode enabled")
                    else:
                        rospy.logerr("Failed to set AUTO.LAND mode")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")

            self.rate.sleep()

    def kalman_acc_land(self):
        # Kalman filter for position estimation
        kalman_filter = KalmanFilter_acc(dt=0.1)

        while not rospy.is_shutdown():
            vel_cmd = Twist()
            k = 0.01

            # Get the Aruco and Yolo data (measurements)
            z_aruco = np.array([self.aruco_x_error, self.aruco_y_error])
            z_yolo = np.array([self.yolo_x_error, self.yolo_y_error])

            # Kalman filter prediction step
            kalman_filter.predict()

            # Kalman filter update with both Aruco and Yolo data
            kalman_filter.update(z_aruco)
            kalman_filter.update(z_yolo)

            # Get the estimated state (center position)
            estimated_position = kalman_filter.get_state()

            # Control based on the Kalman filter output
            print('Estimated Position = ', estimated_position)
            print('Altitude = ', self.rel_alt.data)

            cp_x = self.pids['x'](estimated_position[0])
            cp_y = self.pids['y'](estimated_position[1])

            # Log and save the data
            self.update_csv(cp_x, cp_y, self.rel_alt.data)

            vel_cmd.linear.x = k * cp_y
            vel_cmd.linear.y = k * cp_x
            vel_cmd.linear.z = -0.3
            self.vel_pub.publish(vel_cmd)

            if abs(self.rel_alt.data - 1) < 0.01:
                vel_cmd.linear.x = 0.0
                vel_cmd.linear.y = 0.0
                vel_cmd.linear.z = 0.0
                self.vel_pub.publish(vel_cmd)

                rospy.loginfo("Setting AUTO.LAND mode...")
                land_set_mode = SetModeRequest()
                land_set_mode.custom_mode = "AUTO.LAND"
                try:
                    if self.set_mode_client.call(land_set_mode).mode_sent:
                        rospy.loginfo("AUTO.LAND mode enabled")
                    else:
                        rospy.logerr("Failed to set AUTO.LAND mode")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")

            self.rate.sleep()

    def fly_line(self, u, v, w, t):
        vel_cmd = Twist()
        vel_cmd.linear.x = u
        vel_cmd.linear.y = v
        vel_cmd.linear.z = w
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < t:
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        self.vel_pub.publish(vel_cmd)

    def fly_line_with_yaw(self, u, v, w, yaw_rate, t):
        vel_cmd = Twist()
        vel_cmd.linear.x = u
        vel_cmd.linear.y = v
        vel_cmd.linear.z = w
        vel_cmd.angular.z = yaw_rate  # Yaw radians per second
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < t:
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        vel_cmd.linear.x = 0.0
        vel_cmd.linear.y = 0.0
        vel_cmd.linear.z = 0.0
        vel_cmd.angular.z = 0.0  # Stop yaw rotation
        self.vel_pub.publish(vel_cmd)


    def rotate_drone(self, yaw_rate, duration):
        vel_cmd = Twist()
        vel_cmd.angular.z = yaw_rate  # Yaw rate in radians per second
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()
        vel_cmd.angular.z = 0.0  # Stop yaw rotation
        self.vel_pub.publish(vel_cmd)




if __name__ == "__main__":
    ### check method_name
    controller = OffboardController(method_name="test")


    controller.set_mav_frame(8)

    ### Takeoff
    controller.takeoff(alt=10.0)

    ### Control
    # controller.fly_line(1, 0, 0, 3.0)

    ### Control with yaw
    controller.fly_line_with_yaw(1, 0, 0, 0.5, 3.0)

    ### Rotate drone : Rotate at 0.1 rad/s for 5 seconds, 90 deg = 1.5708 rad
    #controller.rotate_drone(yaw_rate=1.507, duration=2)
    
    # for times in range(3):
    #     controller.fly_line(1, 0, 0.2, 3.0)
    #     controller.rotate_drone(yaw_rate=1.5708, duration=1)
    #     controller.fly_line(1, 0, 0.2, 3.0)
    #     controller.rotate_drone(yaw_rate=1.5708, duration=1)
        
        

    ### Land
    # controller.land()
    # controller.aruco_land()
    # controller.yolo_land()
    # controller.kalman_land()
    controller.kalman_acc_land()
    controller.close_csv()