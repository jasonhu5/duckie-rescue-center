#!/usr/bin/env python
import yaml
import numpy
import math
import time
from matplotlib import pyplot as plt
from simple_map import SimpleMap

map_file = "map.yaml"
map = SimpleMap(map_file)
map.display_raw_debug()

def plot_exit(map, current_pos, heading, target_pos, target_heading):
    # use binary map for shape
    num_y = map.map_bin.shape[0] # reverse axis
    num_x = map.map_bin.shape[1] # reverse axis
    delta = map.tile_size
    fig, ax = plt.subplots()
    # Plot tiles
    for i in range(num_x+1):
        plt.axvline(x=i*delta)
    for j in range(num_y+1):
        plt.axhline(y=j*delta)
    for i in range(num_x):
        for j in range(num_y):
            if map.map_bin[i,j]:
                x = [i*delta, i*delta, (i+1)*delta, (i+1)*delta]
                y = [j*delta, (j+1)*delta, (j+1)*delta, j*delta]
                ax.fill(x, y, color='gray')
    # Plot formatting
    ax.invert_yaxis()
    ax.xaxis.tick_top()
    ax.set_ylabel('x')
    ax.set_xlabel('y')
    ax.set_xlim([0,num_x*delta])
    ax.set_ylim([num_y*delta,0])
    # Plot points and heading
    plt.plot(current_pos[1], current_pos[0], 'o') # current_pos
    angle = heading*math.pi/180
    plt.arrow(current_pos[1], current_pos[0],
              delta/4*math.sin(angle), delta/4*math.cos(angle))
    if target_pos is not None and isinstance(target_pos, tuple):
        plt.plot(target_pos[1], target_pos[0], '*') # target_pos
        target_angle = target_heading*math.pi/180
        plt.arrow(target_pos[1], target_pos[0],
                  delta/4*math.sin(target_angle), delta/4*math.cos(target_angle))
    plt.show()

test_pos = (1.5, 3)
print(map.pos_to_semantic(test_pos))
test_heading = 43
print("Test position: ", test_pos)
print("Test heading: ", test_heading)
t1 = time.time()
target_position = map.pos_to_ideal_position(test_pos, test_heading)
t2 = time.time()
target_heading = map.pos_to_ideal_heading(target_position)
t3 = time.time()
print("Target position: {} (found in {} ms)".format(target_position, 1000*(t2-t1)))
print("Target heading: {} (found in {} ms)".format(target_heading, 1000*(t3-t2)))
plot_exit(map, test_pos, test_heading, target_position, target_heading)






# OLD STUFF

# test_pos_1 = (0.2, 2.1)
# test_pos_2 = (0.7, 2.93)
# test_pos_3 = (3.11, 0.58)
# test_pos_4 = (2.4, 3.4)
# print("Tile_size =", map.tile_size)
# #map.display_sem_debug()
# # print("- Test position:")
# # print(test_pos)
# # print("- Position on map:")
# # print(map.position_on_map(test_pos, subtile=True))
# print("Heading from positions:")
# print(test_pos_1)
# print(map.pos_to_ideal_position(test_pos_1))
# print(test_pos_2)
# print(map.pos_to_ideal_position(test_pos_2))
# print(test_pos_3)
# print(map.pos_to_ideal_position(test_pos_3))
# print(test_pos_4)
# print(map.pos_to_ideal_position(test_pos_4))