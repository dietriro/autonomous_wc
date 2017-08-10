import numpy as np
from numpy import pi
from Node import Node
from copy import copy

# a = 6.5
#
# while a > 2*pi:
#     a -= 2*pi
#
# while a < 0:
#     a += 2*pi
#
# print(a)

# a = np.load('/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d.npy')
#
# print(np.argwhere(a > 0))
# # print(np.where(a > 0))
#
# print(a[a>0])
#
# a[1, 0, 0] = 10
# a[0, 1, 0] = 20
# a[0, 0, 1] = 30
#
# a = a.astype(np.int32)
#
# np.save('/home/robin/catkin_ws/src/osu_research/autonomous_wc/data/costmap3d.npy', a)


a = Node(1)

b = copy(a)
a.id = 3
print(b.id)