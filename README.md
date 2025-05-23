## Offboard Python Scripts

This folder contains custom ROS Python scripts for drone simulation, control, and perception, including ArUco marker and YOLO object detection, offboard control, and data logging.

### Scripts

- **detect_aruco.py**  
  Detects ArUco markers in images from a ROS camera topic, estimates their position relative to the camera, and publishes the X and Y pixel offsets of the marker center.  
  - Subscribes to: `/DIDALOS_vtol/usb_cam/image_raw` (sensor_msgs/Image)
  - Publishes: `aruco_center_x` and `aruco_center_y` (std_msgs/Float32)
  - Uses OpenCV's ArUco module for marker detection.
  - Displays the processed image with marker outlines and error vectors.

- **detect_yolo.py**  
  Runs YOLOv5 object detection on images from a ROS topic, publishing detected bounding boxes and optionally annotated images.  
  - Subscribes to: configurable image topic (raw or compressed)
  - Publishes: `detections` (BoundingBoxes), optionally annotated images
  - Uses YOLOv5 for detection; supports GPU/CPU and configurable parameters via ROS.
  - Prints and logs detected object center offsets.

- **drone.py**  
  Provides a `DroneController` class for controlling a drone in offboard mode, handling state updates, arming, mode switching, and processing ArUco marker position callbacks.  
  - Subscribes to: `aruco_distance_error_x`, `aruco_distance_error_y` (std_msgs/Float32), `/mavros/state`
  - Publishes: `/mavros/setpoint_velocity/cmd_vel` (TwistStamped)
  - Contains methods for arming, setting flight modes, and responding to marker detections.

- **offb_node.py**  
  Minimal example for offboard position control using MAVROS.  
  - Subscribes to: `/mavros/state`
  - Publishes: `/mavros/setpoint_position/local` (PoseStamped)
  - Arms the drone and switches to OFFBOARD mode, holding a fixed position.

- **postition_logger.py**  
  Logs the drone's local position to a CSV file for later analysis.  
  - Subscribes to: `/mavros/local_position/pose` (PoseStamped)
  - Saves timestamped position data (`x`, `y`, `z`) to `~/data/position_data.csv`.

- **winter.py**  
  Advanced offboard controller supporting ArUco/YOLO-based landing and Kalman filter fusion.  
  - Subscribes to: `/mavros/state`, `/mavros/global_position/rel_alt`, `/mavros/local_position/pose`, `aruco_center_x`, `aruco_center_y`, `yolo_center_x`, `yolo_center_y`
  - Publishes: `/mavros/setpoint_position/local`, `/mavros/setpoint_velocity/cmd_vel_unstamped`
  - Implements PID-based landing using ArUco or YOLO, and Kalman filter fusion of both.
  - Supports takeoff, line/yaw flight, and landing.

- **winter2.py**  
  Similar to `winter.py` but adds CSV logging of control and position data, and includes an extended Kalman filter with acceleration.  
  - Subscribes/publishes: same as `winter.py`
  - Logs landing data to `~/data/landing_data_<method>.csv`
  - Supports multiple landing strategies: ArUco, YOLO, Kalman, and Kalman with acceleration.

### Usage

1. Make sure ROS and all dependencies (cv_bridge, OpenCV, numpy, torch, simple_pid, etc.) are installed.
2. Launch your ROS core and camera driver.
3. Run the scripts as ROS nodes, e.g.:
   ```sh
   rosrun offboard_py detect_aruco.py
   rosrun offboard_py detect_yolo.py
   rosrun offboard_py drone.py
   rosrun offboard_py offb_node.py
   rosrun offboard_py postition_logger.py
   rosrun offboard_py winter.py
   rosrun offboard_py winter2.py
   ```

### Notes

- Camera calibration parameters, marker size, and YOLO model weights can be set via ROS parameters.
- These scripts are intended for use in a ROS Catkin workspace as part of a drone simulation and control stack.
- Some scripts require additional ROS messages or custom message types (e.g., `detection_msgs/BoundingBoxes`).
