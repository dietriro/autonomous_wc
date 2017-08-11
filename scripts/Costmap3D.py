import rospy
import numpy as np

class Costmap3D:
    def __init__(self, resolution, size, angle_res=4, values=None):
        self.resolution = resolution
        self.angle_res = angle_res
        self.size = size
        
        if values is None:
            self.values = np.zeros((size[0], size[1], angle_res))
        else:
            self.values = values
        
    def get_cell_index(self, x, y, t):
        
        if x < 0 or x >= self.size[0] or y < 0 or y >= self.size[1]:
            rospy.loginfo('Value out of map bounds!')
            return None

        return int(np.floor(x/self.resolution)), int(np.floor(y/self.resolution)), self.get_angular_index(t)
    
    def get_cell_value(self, x, y, t):
        index = self.get_cell_index(x, y, t)
        if index is None:
            return False
        return self.values[index[0], index[1], index[2]]
        
    def set_cell_value(self, x, y, t, value):
        index = self.get_cell_index(x, y, t)
        if index is None:
            return False
        self.values[index[0], index[1], index[2]] = value
        
    def inc_cell_value(self, x, y, t, value):
        index = self.get_cell_index(x, y, t)
        if index is None:
            return False
        
        if index[2] == 0:
            lower_angle = 3
            upper_angle = index[2]+1
        elif index[2] == 3:
            lower_angle = index[2]-1
            upper_angle = 0
        else:
            lower_angle = index[2]-1
            upper_angle = index[2]+1

        # self.values[index[0], index[1], index[2]] += value
        
        # # First round
        # self.values[index[0]+1, index[1], index[2]] += value/2
        # self.values[index[0]-1, index[1], index[2]] += value/2
        # self.values[index[0], index[1]+1, index[2]] += value/2
        # self.values[index[0], index[1]-1, index[2]] += value/2
        # self.values[index[0]+1, index[1]-1, index[2]] += value/4
        # self.values[index[0]+1, index[1]+1, index[2]] += value/4
        # self.values[index[0]-1, index[1]-1, index[2]] += value/4
        # self.values[index[0]-1, index[1]+1, index[2]] += value/4
        #
        # # Second round
        # self.values[index[0]+2, index[1], index[2]] += value/4
        # self.values[index[0]-2, index[1], index[2]] += value/4
        # self.values[index[0], index[1]+2, index[2]] += value/4
        # self.values[index[0], index[1]-2, index[2]] += value/4
        # self.values[index[0]+1, index[1]+2, index[2]] += value/4
        # self.values[index[0]+1, index[1]-2, index[2]] += value/4
        # self.values[index[0]-1, index[1]+2, index[2]] += value/4
        # self.values[index[0]-1, index[1]-2, index[2]] += value/4
        # self.values[index[0]+2, index[1]+1, index[2]] += value/4
        # self.values[index[0]+2, index[1]-1, index[2]] += value/4
        # self.values[index[0]-2, index[1]+1, index[2]] += value/4
        # self.values[index[0]-2, index[1]-1, index[2]] += value/4
        #
        # # Different angles
        # self.values[index[0], index[1], lower_angle] += value/4
        # self.values[index[0], index[1], upper_angle] += value/4

        # self.values[index[0]+1, index[1], index[2]] += value/2
        # self.values[index[0]-1, index[1], index[2]] += value/2
        # self.values[index[0], index[1]+1, index[2]] += value/2
        # self.values[index[0], index[1]-1, index[2]] += value/2
        # self.values[index[0]+1, index[1]-1, index[2]] += value/4
        # self.values[index[0]+1, index[1]+1, index[2]] += value/4
        # self.values[index[0]-1, index[1]-1, index[2]] += value/4
        # self.values[index[0]-1, index[1]+1, index[2]] += value/4

        r = 20

        for x in range(-r/2, r/2+1):
            for y in range(-r/2, r/2+1):
                cell = np.array([index[0]+x, index[1]+y, index[2]])
                self.values[cell[0], cell[1], cell[2]] += 1 / (np.linalg.norm(cell-index) + 1) * value
        
        # Different angles
        self.values[index[0], index[1], lower_angle] += value/4
        self.values[index[0], index[1], upper_angle] += value/4


    def get_angular_index(self, angle):
        angle = normalize_angle(angle)
        
        if angle > 0.25 * np.pi:
            if angle > 0.75 * np.pi:
                if angle > 1.25 * np.pi:
                    if angle > 1.75 * np.pi:
                        index = 0
                    else:
                        index = 3
                else:
                    index = 2
            else:
                index = 1
        else:
            index = 0
            
        return index
    
    def is_valid(self, pose):
        '''
        Checks whether the given pose is within the bounds and not occupied.
        :param pose: The pose in the map to check for validity.
        :return: True if the given pose is within the bounds and not occupied, False otherwise.
        '''
        # Check for map bounds
        if pose[0] < 0 or pose[0] >= self.values.shape[0] or \
                pose[1] < 0 or pose[1] >= self.values.shape[1] or \
                pose[2] < 0 or pose[2] >= self.values.shape[2]:
            return False
        
        # Check for occupancy of pose
        if self.values[pose[0], pose[1], pose[2]] > 252:
            return False
        
        return True
                        
                        
        
        

def normalize_angle(angle):
    while angle > 2 * np.pi:
        angle -= 2 * np.pi
    
    while angle < 0:
        angle += 2 * np.pi
        
    return angle