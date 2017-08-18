#!/usr/bin/env python

# ROS
import rospy
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PoseArray
from nav_msgs.msg import Path
from tf import transformations
from autonomous_wc.srv import *
# Local
from AStar3D import AStar3D
from Costmap3D import Costmap3D

import numpy as np
import scipy.io as sio

last_start = None
last_goal = None
last_path = None


def pose_to_pose2d(pose_stamped):
    # Convert quaternion to euler angles
    quaternion = (
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)

    # Create 2D-Pose
    pose2d = Pose2D()
    pose2d.x = pose_stamped.pose.position.x
    pose2d.y = pose_stamped.pose.position.y
    pose2d.theta = euler[2]

    return pose2d


def pose2d_to_pose(pose2d):
    # Convert euler angles to quaternion
    quaternion = transformations.quaternion_from_euler(0, 0, pose2d.theta)
    
    # Create Pose
    pose = Pose()
    pose.position.x = pose2d.x
    pose.position.y = pose2d.y
    pose.position.z = 0
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose


def handle_get_path(req):
    global last_path, last_goal, last_start, pub_path
    print('Service called, path planner is executed.')
    
    # Initialize np map
    exp_map_values = np.load('/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d_sim.npy')
    exp_map = Costmap3D(req.map.resolution, [req.map.width, req.map.height], values=exp_map_values)
    cost_map = np.zeros((req.map.width, req.map.height), int)
    print(cost_map.shape)
    
    # Check for old path
    start = pose_to_pose2d(req.start)
    goal = pose_to_pose2d(req.goal)
    
    if last_path is not None:
        if exp_map.get_cell_index(start.x, start.y, start.theta) == exp_map.get_cell_index(last_start.x, last_start.y, last_start.theta) and \
                        exp_map.get_cell_index(goal.x, goal.y, goal.theta) == exp_map.get_cell_index(last_goal.x, last_goal.y, last_goal.theta):
            print('Returned last path')
            return last_path
    
    # Convert 2d-array map to np map
    for y in range(req.map.height):
        for x in range(req.map.width):
            cost_map[x, y] = req.map.data[y*req.map.width+x]

    # TODO: Set correct threshold
    # cost_map[cost_map < 200] = 0
    # cost_map[cost_map == 255] = 0

    for i in range(exp_map.angle_res):
        exp_map.values[:, :, i] += cost_map
        
    exp_map.values[exp_map.values<0] = 0
    exp_map.values[exp_map.values>254] = 254

    sio.savemat('/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d_sim', {'exp_map': exp_map.values, 'cost_map': cost_map})

    # mat = sio.loadmat('/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d_modified')
    # exp_map.values = mat['map']
    
    # Create path planner node
    path_planner = AStar3D(exp_map, r_max=500, t_factor=0)
    
    # Calculate path for given start and goal
    path = path_planner.calculate_path(pose_to_pose2d(req.start),
                                       pose_to_pose2d(req.goal))
    
    if path is None:
        return PoseArray()
    
    # print(path)
    
    out_path = Path()
    out_path.header.stamp = rospy.get_rostime()
    out_path.header.frame_id = req.start.header.frame_id
    
    pose_path = PoseArray()
    pose_path.header.stamp = rospy.get_rostime()
    pose_path.header.frame_id = req.start.header.frame_id
    pose_path.header.seq = 1
    
    seq = 0
    
    for pose2d in path:
        seq += 1
        
        # Create new PoseStamped from Pose2D
        new_pose_st = PoseStamped()
        new_pose_st.header = out_path.header
        new_pose_st.header.seq = seq
        new_pose = Pose()

        new_pose = pose2d_to_pose(pose2d)
        new_pose.position.x *= 0.05
        new_pose.position.y *= 0.05
        
        new_pose_st.pose = new_pose
        
        # print(new_pose.position)

        # Append it to the pose path
        pose_path.poses.append(new_pose)
        out_path.poses.append(new_pose_st)
        
    pub_path.publish(out_path)
        
    print('Service successfully completed.')
    
    last_start = start
    last_goal = goal
    last_path = pose_path
    
    return pose_path


if __name__ == '__main__':
    ##### ROS #####
    # Initialize this node
    rospy.init_node('sdm_path_planner', anonymous=True)
    
    # Service
    path_planner_srv = rospy.Service('AStar3D_Global_Planner', AStar3D_Global_Planner, handle_get_path)
    
    # Subscriber
    # sub_sensor_data = rospy.Subscriber('/robot0/laser_0', LaserScan, cb_sensor_data)
    
    # Publisher
    pub_path = rospy.Publisher('/global_path', Path, queue_size=10)
    
    # Register function that is called before shutdown
    # rospy.on_shutdown(sess.close)
    
    print('Service established and waiting for calls.')
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()