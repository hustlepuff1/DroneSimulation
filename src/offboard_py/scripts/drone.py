import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Float32
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest, SetMavFrame
from geometry_msgs.msg import TwistStamped
import math

class DroneController:
    def __init__(self):
        rospy.init_node("drone_controller")

        self.aruco_x_error = 0.0
        self.aruco_y_error = 0.0
        self.aruco_x_sub = rospy.Subscriber("aruco_distance_error_x", Float32, self.aruco_x_callback)
        self.aruco_y_sub = rospy.Subscriber("aruco_distance_error_y", Float32, self.aruco_y_callback)

        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.state = State()

        self.cmd_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(20)  # 20Hz

    def aruco_x_callback(self, msg):
        self.aruco_x_error = msg.data

    def aruco_y_callback(self, msg):
        self.aruco_y_error = msg.data

    def state_callback(self, msg):
        self.state = msg

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_service(True)
        except rospy.ServiceException as e:
            rospy.logerr("Arming failed: %s", str(e))

       
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

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_mode_service(0, mode)  # 0 is custom mode
        except rospy.ServiceException as e:
            rospy.logerr("Set mode failed: %s", str(e))

    def offboard_mode(self):
        while not rospy.is_shutdown() and not self.state.connected:
            self.rate.sleep()

        rospy.loginfo("Waiting for mode change...")
        self.set_mode("OFFBOARD")
        rospy.loginfo("Offboard mode enabled.")


    def control_drone(self):
        self.offboard_mode()
        self.arm()

        while not rospy.is_shutdown():
            vel_cmd = TwistStamped()
            vel_cmd.header.stamp = rospy.Time.now()
            vel_cmd.twist.linear.x = self.aruco_y_error * -0.01  # Compensate for x error
            #self.cmd_pub.publish(vel_cmd)

                       
            rospy.sleep(0.05)  # Wait for 1 second

            vel_cmd.twist.linear.y = self.aruco_x_error * 0.01  # Compensate for y error
            self.cmd_pub.publish(vel_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.set_mav_frame(8)
        controller.control_drone()
       

    except rospy.ROSInterruptException:
        pass