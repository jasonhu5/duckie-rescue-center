#!/bin/bash

set -e
roslaunch rescue-center rescue_center.launch veh:=$AUTOBOT_NAME
