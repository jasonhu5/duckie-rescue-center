#!/bin/bash

set -e

git clone https://github.com/$MAP_FORK/duckietown-world.git /duckietown-world
roslaunch rescue_center rescue_center.launch 
