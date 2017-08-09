#!/usr/bin/env python

from Costmap3D import Costmap3D
import numpy as np

import rospy
from tf import transformations
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

data_path = '/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d.npy'
resolution = None
size = None
exp_map = None
inc_value = 1.0


def quat_to_euler(orientation):
    # Convert from quaternion to euler angle
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    
    return euler[2]


def cb_costmap(msg):
    global resolution, size, exp_map
    
    resolution = msg.info.resolution
    size = [msg.info.width, msg.info.height]
    
    exp_map = Costmap3D(resolution, size, angle_res=4)

    rospy.loginfo('Parameters received and 3D costmap initialized. Ready to drive!')
    
    
def cb_pose_update(msg):
    global exp_map, inc_value , pub_progress
    
    if exp_map is not None:
        exp_map.inc_cell_value(msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_euler(msg.pose.pose.orientation), inc_value)
        # exp_map.inc_cell_value(msg.pose.pose.position.x+1, msg.pose.pose.position.y, quat_to_euler(msg.pose.pose.orientation), inc_value/4)
        # exp_map.inc_cell_value(msg.pose.pose.position.x-1, msg.pose.pose.position.y, quat_to_euler(msg.pose.pose.orientation), inc_value/4)
        # exp_map.inc_cell_value(msg.pose.pose.position.x, msg.pose.pose.position.y+1, quat_to_euler(msg.pose.pose.orientation), inc_value/4)
        # exp_map.inc_cell_value(msg.pose.pose.position.x, msg.pose.pose.position.y-1, quat_to_euler(msg.pose.pose.orientation), inc_value/4)

        rospy.loginfo('Got a new pose. Updated costmap!')
        
        # DEBUG
        value = exp_map.get_cell_value(msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_euler(msg.pose.pose.orientation))
        index = exp_map.get_cell_index(msg.pose.pose.position.x, msg.pose.pose.position.y, quat_to_euler(msg.pose.pose.orientation))
        
        rospy.logdebug('New Cell Value [%i, %i, %i]  =  %f', index[0], index[1], index[2], value)

        values = exp_map.values[:, :, 0] + exp_map.values[:, :, 1] + exp_map.values[:, :, 2] + exp_map.values[:, :, 3]

        values /= values.max()
        values *= 100

        og = OccupancyGrid()
        og.info.height = 200
        og.info.width = 200
        og.info.resolution = 0.05
        og.data = values.astype(np.int8).transpose().flatten()

        pub_progress.publish(og)
    else:
        rospy.loginfo('Waiting for initialization of costmap.')
        

def save_data():
    global data_path, exp_map
    
    max_value = 120
    
    exp_map.values = exp_map.values / np.max(exp_map.values) * max_value
    exp_map.values = max_value/2 - exp_map.values
    
    np.save(data_path, exp_map.values)


if __name__ == '__main__':
    
    ##### ROS #####
    
    # Initialize Node
    rospy.init_node('experience_gathering')
    
    # Publisher
    pub_progress = rospy.Publisher('/astar', OccupancyGrid, queue_size=10)

    # Subscriber
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, cb_costmap)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, cb_pose_update)
    
    rospy.loginfo('Initialized ROS successfully!')
    
    rospy.on_shutdown(save_data)
    
    rospy.spin()