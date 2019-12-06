#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
import yaml

class SimpleMap():
    def __init__(self, map_file):
        # Read yaml file
        with open(map_file, 'r') as stream:
            try:
                complete_dict = yaml.safe_load(stream)
                # Add vertices for all the floor tags.
                for key, value in complete_dict.iteritems():
                    if key == "tiles":
                        map_raw = value
                    if key == "tile_size":
                        tile_size = value
            except yaml.YAMLError as exc:
                print(exc)
        # Tile size
        self.tile_size = tile_size
        # Raw string version from yaml file
        self.map_raw = map_raw
        # Convert to binary on road/off road
        self.map_bin = self.raw2bin(self.map_raw)
        # Convert to state
        self.map_state = self.raw2state(self.map_raw)


    def raw2bin(self, map_raw):
        # Very simple function that converts raw map to binary on road/off road
        # 1 = street
        # 0 = no street
        map_bin = np.empty_like(np.array(map_raw))
        numrows = len(map_raw)
        numcols = len(map_raw[0])
        for row in range(numrows):
            for col in range(numcols):
                map_bin[row][col]  = 0 if map_raw[row][col] == 'asphalt' else 1
        return map_bin

    def raw2state(self, map_raw):
        # Very simple function that converts the raw map to lane/intersection/out_of_lane
        map_state = np.empty_like(np.array(map_raw))
        numrows = len(map_raw)
        numcols = len(map_raw[0])
        for row in range(numrows):
            for col in range(numcols):
                if 'asphalt' in map_raw[row][col]:
                    map_state[row][col]  = 'out_of_lane'
                elif 'way' in map_raw[row][col]:
                    map_state[row][col]  = 'intersection'
                else:
                    map_state[row][col]  = 'lane'
        return map_state

    def position_on_map(self, position, map, subtile=False):
        # Returns higher-level information on the position_on_map
        # depending on the map passed as argument (bin, state, etc.)
        if subtile:
            return None
        else:
            tile_x = int(math.floor(position[0]/self.tile_size))
            tile_y = int(math.floor(position[1]/self.tile_size))
            return map[tile_x, tile_y]
            
    def display_debug(self):
        # Display map for debugging
        numrows = len(self.map_raw)
        numcols = len(self.map_raw[0])
        for row in range(numrows):
            print(' ')
            for col in range(numcols):
                if self.map_raw[row][col] == 'asphalt':
                    print 'O',
                elif self.map_raw[row][col] == 'straight/S' or self.map_raw[row][col] == 'straight/N':
                    print '|',
                elif self.map_raw[row][col] == 'straight/W' or self.map_raw[row][col] == 'straight/O':
                    print '-',
                else:
                    print '+',