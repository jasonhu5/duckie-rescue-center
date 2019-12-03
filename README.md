# Rescue center worker

How to build:
```
dts devel build -f --arch amd64
```

How to run:
```
docker run --name rescue_center_27 --network=host -it --rm -e AUTOBOT_NAME=autobot27 -e ROS_MASTER_IP=http://192.168.1.36:11311 duckietown/duckie-rescue-center:v1-amd64
```
