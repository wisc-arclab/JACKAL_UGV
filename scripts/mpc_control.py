#!/usr/bin/env python3
# acados
from acados_template import AcadosOcpSolver
from create_ocp import create_ocp
# ros
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
# python
import time
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

# initialize solver
ocp = create_ocp()
N = ocp.dims.N
solver = AcadosOcpSolver(ocp)

# initialize ocp
x = np.zeros((N + 1, 3))
u = np.zeros((N + 1, 2))
yref = np.zeros((N + 1, 5))
for i in range(N):
    solver.cost_set(i, "yref", yref[i])
solver.cost_set(N, "yref", yref[N][:3])
for i in range(N + 1):
    solver.set(i, 'x', np.zeros(3))
x0 = np.zeros((3,))
solver.constraints_set(0, "lbx", x0)
solver.constraints_set(0, "ubx", x0)
solver.solve()

# receive desired trajectory
received_traj = False
def path_callback(msg):
    """Callback for the path message."""
    global received_traj, path_points
    if not received_traj:
        path_points = [[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses]
        received_traj = True
        rospy.loginfo("Received new path points with %d poses.", len(msg.poses))
    #for i, point in enumerate(path_points):
    #    rospy.loginfo("Point %d: x = %f, y = %f", i, point[0], point[1])

# receive odom for x0
def odom_callback(msg):
    """Callback for the odometry message."""
    global current_odom
    current_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z])
    #rospy.loginfo("Received x0.")

start_index = 0
def mpc_control(event):

    global path_points, current_odom, start_index
    
    path = path_points
    odom = current_odom
    
    if path is None or odom is None:
        rospy.loginfo("Waiting for path or odom...")
        return
        
    start_time = time.time()
    rospy.loginfo("Starting mpc")
    
    # set cost weight
    W1 = np.diag([100, 100, 1, 1, 1])
    W2 = np.diag([100, 100, 1])
    for i in range(N):
        solver.cost_set(i, 'W', W1)
    solver.cost_set(N, 'W', W2)
    
    # set ref  
    if start_index + N + 1 <= len(path_points):
        x_pts = np.array([point[0] for point in path_points[start_index:start_index + 21]])
        y_pts = np.array([point[1] for point in path_points[start_index:start_index + 21]])
    else:
        rospy.loginfo("Not enough points in path_points")
        return
        
    #if len(path_points) == N + 1:
    #    rospy.loginfo("solver got ref points")
    #    x_pts = np.array([point[0] for point in path_points])
    #    y_pts = np.array([point[1] for point in path_points])
    #else:
    #    return
    
    yref[:, 0] = x_pts
    yref[:, 1] = y_pts
    #yref[:, 2] = theta_pts
    for i in range(N):
        solver.cost_set(i, "yref", yref[i])
    solver.cost_set(N, "yref", yref[N][:3])
    
    # set x0
    x0 = current_odom
    solver.constraints_set(0, "lbx", x0)
    solver.constraints_set(0, "ubx", x0)
    
    # solve
    solution_status = solver.solve()
    rospy.loginfo("MPC solution status: %d", solution_status)
    

    # Publish command velocities
    u = solver.get(0, 'u')
    cmd_msg = Twist()
    cmd_msg.linear.x = u[0]  # Assuming first control input is linear velocity
    cmd_msg.angular.z = u[1]  # Assuming second control input is angular velocity
    cmd_vel_publisher.publish(cmd_msg)
    
    # Publish predicted trajectory
    predicted_path = Path()
    predicted_path.header.stamp = rospy.Time.now()
    predicted_path.header.frame_id = "odom"  # or the appropriate frame
    
    for i in range(N + 1):
        state = solver.get(i, 'x')
        pose = PoseStamped()
        pose.pose.position.x = state[0]
        pose.pose.position.y = state[1]
        pose.pose.orientation.z = state[2]  # or convert to a full quaternion if needed
        predicted_path.poses.append(pose)

    predict_path_publisher.publish(predicted_path)
    
    # Publish segment points
    segment_path = Path()
    segment_path.header.stamp = rospy.Time.now()
    segment_path.header.frame_id = "odom"
    
    for x, y in zip(x_pts, y_pts):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0  # Assuming a flat trajectory
        segment_path.poses.append(pose)

    seg_points_publisher.publish(segment_path)
    
    # Update start index for next control cycle
    start_index += 1
    if start_index + 21 > len(path_points):
        rospy.loginfo("Reached the end of the trajectory.")
        start_index = 0  # Reset to start if desired, or stop updating
    
    # print solve time
    end_time = time.time()
    total_time = end_time - start_time
    rospy.loginfo("Total execution time: %f seconds", total_time)
    return
    
if __name__ == '__main__':
    global cmd_vel_publisher
    rospy.init_node('trajectory_tracking_node', anonymous=True)
    #rospy.Subscriber('/nearest_trajectory_segment', Path, path_callback)
    rospy.Subscriber('/trajectory', Path, path_callback)
    rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    predict_path_publisher = rospy.Publisher('/predicted_trajectory', Path, queue_size=10)
    seg_points_publisher = rospy.Publisher('/seg_points', Path, queue_size=10)
    rospy.Timer(rospy.Duration(0.05), mpc_control)
    rospy.spin()
