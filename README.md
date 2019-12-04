# Rescue center worker

chmod +x ./packages/rescue_center/src/rescue_manager_node.py



How to build:
```
dts devel build -f --arch amd64
```

How to run:
```
docker run --name rescue_center --network=host -it --rm -e AUTOBOT_1=27 -e AUTOBOT_2=28  -e ROS_MASTER_IP=http://192.168.1.187:11311 duckietown/duckie-rescue-center:vMartin-amd64
```
