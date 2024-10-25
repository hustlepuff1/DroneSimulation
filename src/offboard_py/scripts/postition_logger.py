#!/usr/bin/env python3

import rospy
import csv
import os
from geometry_msgs.msg import PoseStamped

class PositionLogger:
    def __init__(self):
        # Initialize the node
        rospy.init_node('position_logger_node', anonymous=True)
        
        # Define the path for saving the CSV file
        self.csv_file_path = os.path.expanduser("~/data/position_data.csv")
        
        # Open the CSV file for writing
        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'z'])

        # Subscribe to the position topic (e.g., from mavros)
        self.pos_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.position_callback)
        
        # Start the node
        rospy.loginfo("Position logger started, saving to %s", self.csv_file_path)

    def position_callback(self, msg):
        # Extract position data from the PoseStamped message
        position = msg.pose.position
        timestamp = msg.header.stamp.to_sec()

        # Save the data to the CSV file
        self.csv_writer.writerow([timestamp, position.x, position.y, position.z])

        # Print the position to console for debugging
        rospy.loginfo("Position - x: %.3f, y: %.3f, z: %.3f", position.x, position.y, position.z)

    def close_csv(self):
        # Close the CSV file gracefully
        self.csv_file.close()

if __name__ == '__main__':
    try:
        # Create the logger object
        logger = PositionLogger()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure the CSV file is closed before exiting
        logger.close_csv()
