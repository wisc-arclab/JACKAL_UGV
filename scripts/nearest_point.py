#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class NearestPointFinder:
    def __init__(self):
        rospy.init_node('nearest_point_finder')
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/trajectory', Path, self.path_callback)
        self.path_pub = rospy.Publisher('/nearest_trajectory_segment', Path, queue_size=10)
        self.current_position = None
        self.trajectory = None
        self.received_traj = False

    def odom_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def path_callback(self, msg):
        if not self.received_traj:
            self.trajectory = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
            self.received_traj = True

    def update_trajectory_segment_threshold(self, threshold=0.02, num_points=21, step=2):
        if self.current_position is not None and self.trajectory:
            distances = np.array([np.linalg.norm(self.current_position - np.array([x, y])) for x, y in self.trajectory])
            valid_indices = np.where(distances <= threshold)[0]

            if len(valid_indices) > 0:
                nearest_index = valid_indices[-1]
            else:
                nearest_index = np.argmin(distances)

            start_index = nearest_index
            end_index = min(start_index + num_points * step, len(self.trajectory))
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

            self.path_pub.publish(path_msg)
            rospy.loginfo(f"Published trajectory segment starting at index {nearest_index} with {len(segment)} points.")
        else:
            rospy.loginfo("Waiting for position and trajectory data...")

if __name__ == '__main__':
    finder = NearestPointFinder()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        finder.update_trajectory_segment_threshold()
        rate.sleep()

