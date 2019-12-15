import yaml
import numpy
import math
from matplotlib import pyplot as plt
from simple_map import SimpleMap

######## Copy of local debugging script from Carl, will not work in container ##########

map_file = "map.yaml"
map = SimpleMap(map_file)
map.display_raw_debug()

def plot_exit(map, current_pos, heading, target_pos):
    # use binary map for shape
    num_y = map.map_bin.shape[0] # reverse axis
    num_x = map.map_bin.shape[1] # reverse axis
    delta = map.tile_size
    angle = heading*math.pi/180
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
    plt.arrow(current_pos[1], current_pos[0],
              delta/4*math.sin(angle), delta/4*math.cos(angle))
    if target_pos is not None:
        plt.plot(target_pos[1], target_pos[0], '*') # target_pos
    plt.show()


test_pos = (2.2, 2.2)
test_heading = 0
print("Test position: ", test_pos)
print("Test heading: ", test_heading)
test_target = map.pos_to_ideal_position(test_pos, test_heading)
print("Target position: ", test_target)
plot_exit(map, test_pos, test_heading, test_target)
