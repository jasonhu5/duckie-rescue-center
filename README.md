# Rescue center and rescue agents

## System explanation
In Duckietown, a Duckiebot can run into a few distress situations, e.g. out of lane, stuck with infrastructure, wrong heading in lane. In [Autolabs](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_definition.html), the watchtower system can help monitor the position and heading of the [Autobots](https://docs.duckietown.org/daffy/opmanual_autolab/out/autolab_autobot_specs.html). In this repository, we propose a system that continuously monitor all the Autobots and rescue them back to normal operation (e.g. lane following) if necessary.

The system runs on a lab server machine. The user explicitly run the rescue-center node, and for per new Autobot detected by the [online localization system](https://docs.duckietown.org/daffy/opmanual_autolab/out/localization_demo.html), a rescue-agent node is automatically spawn by the rescue-center, to monitor the behavior of that particular bot, classify distress situation, and perform the rescue operation if needed. The node-wise rostopic publishing/subscribing configurations are commented inline the node source files.

## Commands to build and run

How to build:
```
dts devel build -f --arch amd64
```

How to run:
```
docker run --name rescue_center --network=host -it --rm -e ROS_MASTER_IP=http://<LAB_SERVER_IP>:11311 -e MAP_FORK=<DUCKIETOWN-WORLD_GITHUB_FORK> -e MAP_NAME=<NAME_OF_MAP_WITHOUT_.yaml_SUFFIX> duckietown/duckie-rescue-center:v1-amd64
```

For example:
```
docker run --name rescue_center --network=host -it --rm -e ROS_MASTER_IP=http://127.0.0.1:11311 -e MAP_FORK=jasonhu5 -e MAP_NAME=ethz_amod_lab_k31 duckietown/duckie-rescue-center:v1-amd64
```

## Parameters

### ros parameters (can be changed at run time)

Start a ros container on the labserver with: `docker run -it --rm --net host duckietown/dt-ros-commons:daffy-amd64 /bin/bash`. In the container:

* To add bot to be monitored by rescue system
`rosparam set /rescue/rescue_center/add_duckiebot <AUTOBOT_NUMBER>`
* To remove bot from being monitored by rescue system
`rosparam set /rescue/rescue_center/remove_duckiebot <AUTOBOT_NUMBER>`

### Inline parameters

Some parameters are recommended after experiments, and not expected to change often. They are therefore not made rosparams, but kept as inline parameters. Here is a list stating where these parameters are, and what each does. In case someone wants to tune them.

#### In `packages/rescue_center/src/rescue_agent_node.py`
```python
"""
For the differecne between actual heading and the desired heading, this is the tolerated value. If the difference (absolute value) is smaller, turn on lane following.
"""
TOL_ANGLE_IN_DEG = 30

"""
Similar to above, but for the difference between actual position and the desired position.
"""
TOL_POSITION_IN_M = 0.14625  # tileSize/4

"""
Parameters for planning controller
"""
# proportional gain
KP_CONTROLLER = 6

# integral gain
KI_CONTROLLER = 0.1

# distance coefficient of measurement matrix
# i.e. weight of distance error
C1_CONTROLLER = -5

# heading coefficient of measurement matrix
# i.e. weight of heading error
C2_CONTROLLER = 1

"""
Parameters for rescue strategy execution
"""
# magnitude of rescue linear velocity
# always go with this constant velocity
V_REF_CONTROLLER = 0.3

# For rescue operation, execution and stop times in a cycle
# here: rescue for 0.3s and stop 1s
T_EXECUTION = 0.3
T_STOP = 1

# number of iterations for above exec-stop cycle, as a minimal batch
NUMBER_RESCUE_CMDS = 3
```

#### In `packages/rescue_center/src/autobot_info.py`

These are some distress classification thresholds.

```python
"""
after this number of seconds, if the bot has not moved, it is considered stuck
"""
TIME_DIFF_THRESHOLD = 10

"""
if the difference between the current heading and desired heading (based on the bot position) is larger than this parameter, the bot is considered to have a wrong heading. This could happen, for example, when a Duckiebot runs on to the opposite lane. 
"""
ANGLE_TRHESHOLD = 70
```