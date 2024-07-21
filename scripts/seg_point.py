#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class TrajectorySegmentPublisher:
    def __init__(self):
        rospy.init_node('trajectory_segment_publisher')
        self.odom_subscriber = rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        self.trajectory_subscriber = rospy.Subscriber('/trajectory', Path, self.trajectory_callback)
        self.segment_publisher = rospy.Publisher('/moving_trajectory_segment', Path, queue_size=10)
        self.current_position = None
        self.trajectory = None
        self.has_received_trajectory = False
        self.current_segment_start_index = 0

    def odometry_callback(self, odom_msg):
        self.current_position = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])

    def trajectory_callback(self, path_msg):
        if not self.has_received_trajectory:
            self.trajectory = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
            self.has_received_trajectory = True

    def publish_next_trajectory_segment(self, segment_length=21, step=1):
        if self.trajectory:
            start_index = self.current_segment_start_index
            end_index = min(start_index + segment_length * step, len(self.trajectory))
            segment = self.trajectory[start_index:end_index:step]

            # Create a Path message
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "odom"  # Make sure this matches your frame setup in RViz

            for idx, (x, y) in enumerate(segment):
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0  # Assuming a flat trajectory
                path_msg.poses.append(pose_stamped)
                rospy.loginfo(f"Point {idx + 1}: x={x}, y={y}")

            self.segment_publisher.publish(path_msg)
            rospy.loginfo(f"Published trajectory segment starting at index {start_index} with {len(segment)} points.")

            # Update current_segment_start_index for the next segment
            self.current_segment_start_index = start_index + 1
            if self.current_segment_start_index >= len(self.trajectory):
                rospy.loginfo("Reached the end of the trajectory.")
                self.current_segment_start_index = 0  # Reset to start if desired, or stop updating
        else:
            rospy.loginfo("Waiting for trajectory data...")

if __name__ == '__main__':
    segment_publisher = TrajectorySegmentPublisher()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        segment_publisher.publish_next_trajectory_segment()
        rate.sleep()

