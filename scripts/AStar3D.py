#!/usr/bin/env python

import numpy as np
from Queue import PriorityQueue
from Node import Node
from math import floor, atan2
from geometry_msgs.msg import Pose2D, Pose, PoseArray
from Costmap3D import Costmap3D
import scipy.io as sio
from copy import copy

import rospy
from nav_msgs.msg import OccupancyGrid

class AStar3D:
    '''
    A Class for the calculation of the shortest path using the A-Star Algorithm
    '''
    # The queue containing all nodes that have not been visited so far
    open = None
    # The queue containing all visited nodes
    closed = None
    # The map for the path planning algorithm
    map = None
    # The epsilon value for calculating the heuristic value
    epsilon = None
    # The maximum rate change of the orientation for the robot
    r_max = None
    # The velocity of the robot
    v = None
    # The factor with which the angular sum for a path is multiplied.
    t_factor = None
    # The possible actions within the map
    actions = np.array([[[  1,  0,  0],
                         [  1,  1,  0],
                         [  1, -1,  0],
                         [  0,  0,  1],
                         [  0,  0, -1]],
                        [[  0,  1,  0],
                         [  1,  1,  0],
                         [ -1,  1,  0],
                         [  0,  0,  1],
                         [  0,  0, -1]],
                        [[ -1,  0,  0],
                         [ -1,  1,  0],
                         [ -1, -1,  0],
                         [  0,  0,  1],
                         [  0,  0, -1]],
                        [[  0, -1,  0],
                         [  1, -1,  0],
                         [ -1, -1,  0],
                         [  0,  0,  1],
                         [  0,  0, -1]]])
        
                        # [ 0,  1,  0],   # North
                        # [ 1,  0,  0],   # East
                        # [ 0, -1,  0],   # South
                        # [-1,  0,  0],   # West
                        # [ 0,  0,  1],   # Up
                        # [ 0,  0, -1]])  # Down

    def __init__(self, map, open_size=500000, closed_size=500000, epsilon=1.0,
                 r_max=50.0, v=1.0, t_factor=100.0):
        '''
        Constructor of AStar3D class.
        :param map: The map that the planner is going to use as a numpy array with values between 0 and 1, where 0 means the tile is free and 1 the tile is occupied.
        :param open_size: The maximum size of the open queue.
        :param closed_size: The maximum size of the closed queue.
        :param epsilon: The epsilon value used for calculating the heuristic value.
        :param r_max: The maximum angular change rate allowed.
        :param v: The desired velocity of the robot.
        :param t_factor: The factor with which the angular sum for a path is multiplied.
        '''
        self.map = map
        self.open = PriorityQueue(open_size)
        self.closed = PriorityQueue(closed_size)
        self.back_open = PriorityQueue(open_size)
        self.back_closed = PriorityQueue(closed_size)
        self.epsilon = epsilon
        self.r_max = r_max
        self.v = v
        self.t_factor = t_factor
        
        sio.savemat('map', {'map': self.map})

        self.pub_progress = rospy.Publisher('/astar', OccupancyGrid, queue_size=10)

    # def line_of_sight(self, id_start, id_goal):
    #     '''
    #     Checks the direct path between two nodes for occupancy.
    #     :param id_start: The id of the start node.
    #     :param id_end: The id of the goal node.
    #     :return: True, if the path is not occupied, False otherwise.
    #     '''
    #     pos_s = self.pos_from_id(id_start)
    #     pos_g = self.pos_from_id(id_goal)
    #
    #     xs = int(pos_s[0])
    #     ys = int(pos_s[1])
    #     xe = int(pos_g[0])
    #     ye = int(pos_g[1])
    #
    #     dx = xe - xs
    #     dy = ye - ys
    #
    #     f = int(0)
    #
    #     if dy < 0:
    #         dy = -dy
    #         sy = -1
    #     else:
    #         sy = 1
    #
    #     if dx < 0:
    #         dx = -dx
    #         sx = -1
    #     else:
    #         sx = 1
    #
    #     if dx >= dy:
    #         while xs != xe:
    #             f += dy
    #             if f >= dx:
    #                 if self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
    #                     return False
    #                 ys += sy
    #                 f -= dx
    #             if f != 0 and self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
    #                 return False
    #             if dy == 0 and self.map[int(xs+((sx-1)/2)), int(ys)] and \
    #                     self.map[int(xs+((sx-1)/2)), int(ys-1)]:
    #                 return False
    #             xs += sx
    #     else:
    #         while ys != ye:
    #             f += dx
    #             if f >= dy:
    #                 if self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
    #                     return False
    #                 xs += sx
    #                 f -= dy
    #             if f != 0 and self.map[int(xs+((sx-1)/2)), int(ys+((sy-1)/2))]:
    #                 return False
    #             if dx == 0 and self.map[int(xs), int(ys+((sy-1)/2))] and \
    #                     self.map[int(xs-1), int(ys+((sy-1)/2))]:
    #                 return False
    #             ys += sy
    #     return True
    #
    # def arc_check_angle(self, n_cur, n_next):
    #     o_cur = atan2((n_next.pos[1]-n_cur.pos[1]), (n_next.pos[0]-n_cur.pos[0]))
    #
    #     d = self.h_euc(n_next.pos, n_cur.pos)
    #
    #     alpha = o_cur-n_cur.o
    #
    #     # print(alpha)
    #
    #     # Check if angular rate change is within the max value
    #     if alpha*self.v/d > self.r_max:
    #         # print('FALSE')
    #         return False
    #
    #     # Update new angular change
    #     n_next.o = alpha
    #     n_next.t += self.t_factor*abs(n_next.o)
    #
    #     # print('TRUE')
    #     return True
        
    def get_node(self, index, open, closed):
        '''
        Checks whether a Node with the given id is listed in the queue.
        :param queue: The queue that is searched.
        :param index: The id of the Node that is searched for.
        :return: Either True or False, depending on the success.
        '''
        for n in open.queue:
            if n.id == index:
                return copy(n)
        for n in closed.queue:
            if n.id == index:
                return copy(n)
        return None
    
    def update_node(self, n_cur, n_nbr, open, closed):
        '''
        Updates the g and parent value of a node within the open queue, in case the g value is smaller than the old one.
        :param n_new: The node with the new values.
        :return: True, if the node was found, false if not.
        '''
        g_old = n_nbr.g
        parent = self.get_node(n_cur.parent, open, closed)
        
        # # Check for line of sight to parent
        # if self.line_of_sight(parent.id, n_nbr.id):
        #     self.compute_cost(parent, n_nbr)
        # else:
        self.compute_cost(n_cur, n_nbr)
            
        if n_nbr.g < g_old:
            if n_nbr in open.queue:
                open.queue.remove(n_nbr)
            open.put(Node(n_nbr.id, pos=n_nbr.pos, g=n_nbr.g, h=n_nbr.h,
                               o=n_nbr.o, parent=n_nbr.parent, t=n_nbr.t, c=n_nbr.c))
    
    def compute_cost(self, n_cur, n_nbr):
        g_new = n_cur.g + self.cost(n_cur.pos, n_nbr.pos)# + self.map.values[n_nbr.pos[0], n_nbr.pos[1], n_nbr.pos[2]] * 10000
        if g_new < n_nbr.g:
            n_nbr.parent = n_cur.id
            n_nbr.g = g_new
            # print(n_nbr.t)
            
    def cost(self, start, goal):
        '''
        Calculates the cost between two nodes based on the euclidean distance between them.
        :param id_start: The id of the start node.
        :param id_goal: The id of the goal node.
        :return:
        '''
        return self.h_euc(start, goal) + 20*abs(start[2]-goal[2])
    
    def id_from_pos(self, pos):
        '''
        Returns a unique id for a given (x,y) position in the map.
        :param pos: The position used to generate the unique id.
        :return: The unique id generated.
        '''
        return pos[0] + pos[1]*self.map.values.shape[0] + pos[2]*self.map.values.shape[0]*self.map.values.shape[1]
        
    def pos_from_id(self, index):
        '''
        Calculates the position of a node within the map using the given index.
        :param index: The index of the node within the map.
        :return: The position (x,y) of the node within the map.
        '''
        x = floor(index / self.map.values.shape[1])
        y = index % self.map.values.shape[1]
        return np.array([x, y])
        
    def h_euc(self, start, goal):
        '''
        Returns the heuristic value for the given start point to the goal.
        :param start: The start position in the map.
        :param goal: The goal position in the map
        :return: The euclidean distance to the goal.
        '''
        return np.linalg.norm(start-goal)*self.epsilon
    
    # def is_valid(self, position):
    #     '''
    #     Checks whether the given position is within the bounds and not occupied.
    #     :param position: The position in the map to check for validity.
    #     :return: True if the given position is within the bounds and not occupied, False otherwise.
    #     '''
    #     # Check for map bounds
    #     if position[0] < 0 or position[0] >= self.map.shape[0] or \
    #             position[1] < 0 or position[1] >= self.map.shape[1]:
    #         return False
    #     # Check for occupancy of position
    #     if self.map[position[0], position[1]]:
    #         return False
    #     return True
             
    def get_neighbors(self, node):
        '''
        Finds all valid neighbors for a given node.
        :param node: The node for which to find the neighbors.
        :return: A list of nodes that are neighbors of the given node.
        '''
        nbrs = list()
        node_orientation = node.pos[2]
        for i in range(self.actions.shape[1]):
            new_pose = self.actions[node_orientation, i, :] + node.pos
            
            # Set angular index to maximum if below 0
            if new_pose[2] < 0:
                new_pose[2] = self.map.values.shape[2]-1
            elif new_pose[2] >= self.map.values.shape[2]:
                new_pose[2] = 0
                
            # Check if new node's position is within the map
            if not self.map.is_valid(new_pose):
                continue
                
            # If new position is valid, create a new node using the parent nodes values
            nbrs.append(Node(self.id_from_pos(new_pose), parent=node.id, pos=new_pose,
                             c=self.map.values[new_pose[0], new_pose[1], new_pose[2]]))
        return nbrs

    def get_back_neighbors(self, node):
        '''
        Finds all valid neighbors for a given node.
        :param node: The node for which to find the neighbors.
        :return: A list of nodes that are neighbors of the given node.
        '''
        nbrs = list()
        if node.pos[2] > 1:
            node_orientation = node.pos[2]-2
        else:
            node_orientation = node.pos[2]+2

        for i in range(self.actions.shape[1]):
            new_pose = self.actions[node_orientation, i, :] + node.pos
        
            # Set angular index to maximum if below 0
            if new_pose[2] < 0:
                new_pose[2] = self.map.values.shape[2] - 1
            elif new_pose[2] >= self.map.values.shape[2]:
                new_pose[2] = 0
        
            # Check if new node's position is within the map
            if not self.map.is_valid(new_pose):
                continue
        
            # If new position is valid, create a new node using the parent nodes values
            nbrs.append(Node(self.id_from_pos(new_pose), parent=node.id, pos=new_pose,
                             c=self.map.values[new_pose[0], new_pose[1], new_pose[2]]))
        return nbrs
    
    def get_g(self, index, open, closed):
        '''
        Returns the g value from a node using open/closed queue.
        :param index: The index of the node the g value is requested from.
        :return: The g value if the specified node.
        '''
        for n in open.queue:
            if n.id == index:
                return n.g
        for n in closed.queue:
            if n.id == index:
                return n.g
        return None
   
    def pose2d(self, node):
        '''
        Returns a geometry_msgs::Pose2D for given x,y and theta values.
        :param x: The x value of the Pose2D.
        :param y: The y value of the Pose2D.
        :param theta: The theta value of the Pose2D.
        :return: The Pose2D created from the given arguments.
        '''
        pose2d = Pose2D()
        pose2d.x = node.pos[0]
        pose2d.y = node.pos[1]
        pose2d.theta = node.pos[2]
        
        return pose2d
        
    def publish_progress(self, progress):
        og = OccupancyGrid()
        og.info.height = self.map.values.shape[1]
        og.info.width = self.map.values.shape[0]
        og.info.resolution = 0.05
        og.data = progress.transpose().flatten()
    
        self.pub_progress.publish(og)

    def print_path(self, path, cost):
        '''
        Inserts the path into the map and prints it.
        :param path: The path to be visualized.
        :return: Nothing.
        '''
        map_tmp = np.copy(self.map.values)
        last_pos = None
        length = 0.0
        
        for pose in path:
            # Visualize path on map
            map_tmp[pose.x, pose.y] = 2
            # Calculate path length
            if last_pos is not None:
                length += self.h_euc(last_pos, np.array([pose.x, pose.y]))
            last_pos = np.array([pose.x, pose.y])
            
        print('\nFinal path length:  {0:.2f}'.format(length+cost))
        print(map_tmp)
        print('\n(0) identifies a free cell')
        print('(1) identifies an occupied cell')
        print('(2) identifies a cell belonging to the path')

    def calculate_path(self, start, goal):
        '''
        Calculates the shortest path from start to goal using an AStar3D algorithm.
        :param start: Start position as x/y-coordinates within the map.
        :param goal: Goal position as x/y-coordinates within the map.
        :return: The path as a list of positions within the map from start to goal.
        '''
            
        start = np.array(self.map.get_cell_index(start.x, start.y, start.theta))
        goal = np.array(self.map.get_cell_index(goal.x, goal.y, goal.theta))

        print('START:  ', start)
        print('GOAL:  ', goal)
        
        # Boundary checks
        if not self.map.is_valid(start):
            print('Start position out of map bounds.')
            return
        if not self.map.is_valid(goal):
            print('Goal position out of map bounds.')
            return
        
        progress = np.zeros((self.map.values.shape[0], self.map.values.shape[1]), np.int8)

        found_connection = False
            
        # Save start/goal pos as node
        n_s = Node(self.id_from_pos(start), pos=start, h=self.h_euc(start, goal),
                   # g=self.map.values[start[0], start[1], start[2]],
                   c=self.map.values[start[0], start[1], start[2]])
        n_g = Node(self.id_from_pos(goal), pos=goal)
        
        # Save start/goal pose for other direction
        n_back_s = Node(self.id_from_pos(goal), pos=goal, h=self.h_euc(start, goal),
                        c=self.map.values[goal[0], goal[1], goal[2]])
        n_back_g = Node(self.id_from_pos(start), pos=start)

        
        # Push back start nodes for both directions
        self.open.put(n_s)
        self.back_open.put(n_back_s)
        
        print('Start finding path...')
        
        # Loop until goal reached
        while not self.open.empty() and not self.back_open.empty():
            # Get best node
            n_cur = self.open.get()
            n_back_cur = self.back_open.get()
            
            progress[n_cur.pos[0], n_cur.pos[1]] += 25
            progress[n_back_cur.pos[0], n_back_cur.pos[1]] += 25
            
            self.publish_progress(progress)
            
            if n_cur.id == n_g.id:
                print('GOAL FOUND!')
                break
            
            # Add current node to closed list
            self.closed.put(n_cur)
            self.back_closed.put(n_back_cur)
            
            ### FORWARD
            # Get a list of all neighboring nodes
            nbrs = self.get_neighbors(n_cur)
            
            # Loop through neighboring nodes
            for n_nbr in nbrs:
                if n_nbr in self.back_open.queue:
                    n_back_cur = self.get_node(n_nbr.id, self.back_open, self.back_closed)
                    n_cur = copy(n_nbr)
                    found_connection = True
                    break
                if not n_nbr in self.closed.queue:
                    if not n_nbr in self.open.queue:
                        n_nbr.g = np.inf
                        n_nbr.parent = None
                        n_nbr.h = self.h_euc(n_nbr.pos, goal)
                    self.update_node(n_cur, n_nbr, self.open, self.closed)
                    
            if found_connection:
                break

            ### FORWARD
            # Get a list of all neighboring nodes
            nbrs = self.get_back_neighbors(n_back_cur)

            # Loop through neighboring nodes
            for n_nbr in nbrs:
                if n_nbr in self.open.queue:
                    n_cur = self.get_node(n_nbr.id, self.open, self.closed)
                    n_back_cur = copy(n_nbr)
                    found_connection = True
                    break
                if not n_nbr in self.back_closed.queue:
                    if not n_nbr in self.back_open.queue:
                        n_nbr.g = np.inf
                        n_nbr.parent = None
                        n_nbr.h = self.h_euc(n_nbr.pos, n_g.pos)
                    self.update_node(n_back_cur, n_nbr, self.back_open, self.back_closed)

            if found_connection:
                break
            
        print('Finished path search.')
        
        # Follow path back
        # print('INT PATH: ')
        # print(n_cur.pos)

        n_back_cur.pos[2] *= 0.5*np.pi
        path = [self.pose2d(n_back_cur)]
        cost = 0.0
        while n_back_cur.id != n_g.id:
            # Change current node to parent node
            n_back_cur = self.get_node(n_back_cur.parent, self.back_open, self.back_closed)
            n_back_cur.pos = n_back_cur.pos.astype(float)
            n_back_cur.pos[2] *= 0.5 * np.pi
            # print(n_back_cur.pos)

            # Append current position to path
            path.append(self.pose2d(n_back_cur))
            # Update cost
            cost += n_back_cur.t
        
        path.reverse()
        while n_cur.id != n_s.id:
            # Change current node to parent node
            n_cur = self.get_node(n_cur.parent, self.open, self.closed)
            n_cur.pos = n_cur.pos.astype(float)
            n_cur.pos[2] *= 0.5 * np.pi
            # print(n_cur.pos)

            # Append current position to path
            path.append(self.pose2d(n_cur))
            # Update cost
            cost += n_cur.t
        
        # self.print_path(path, cost)
        
        path.reverse()

        return path
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
            
            
            
    
        
        
