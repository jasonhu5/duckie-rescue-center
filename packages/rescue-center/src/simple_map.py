#!/usr/bin/env python
import os
import math
import rospy
import numpy as np

class SimpleMap():
    def __init__(map_file):
        # Read yaml file
        with open(map_file, 'r') as stream:
            try:
            complete_dict = yaml.safe_load(stream)
            # Add vertices for all the floor tags.
            for key, value in complete_dict.iteritems():
                if key == "tiles":
                    map_raw = value
                if key == "tile_size"
                    tile_size = value
        except yaml.YAMLError as exc:
            print(exc)
        # Tile size
        self.tile_size = tile_size    
        # Raw string version from yaml file
        self.map_raw = map_raw
        # Convert to binary on road/off road 
        self.map_bin = raw2bin(self.map_raw)
        # Convert to dictionary 
        

    def raw2bin(self, map_raw):
        # Very simple function that converts the raw map to on road/off road 
        # 1 = street
        # 0 = no street
        map_bin = np.empty_like(np.array(map_raw))
        numrows = len(map_raw)
        numcols = len(map_raw[0])
        for row in range(numrows):
            for col in range(numcols):
                map_bin[row][col]  = 0 if map_raw[row][col] == 'asphalt' else 1

    def position_on_map_bin(self, position)
        # Returns 1 if the bot is on the road and 0 if it is on an asphalt tile
        tile_x = floor(position[0]/self.tile_size)
        tile_y = floor(position[1]/self.tile_size)
        return self.map_bin[tile_x, tile_y]