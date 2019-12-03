# Rescue center worker

How to build:
```
dts devel build -f --arch amd64
```

How to run:
```
docker run --name rescue_center_<AUTOBOT_NUMBER> --network=host -it --rm -e AUTOBOT_START=27 -e AUTOBOT_NUMBER=2  -e ROS_MASTER_IP=http://<LAB_SERVER_IP>:11311 duckietown/duckie-rescue-center:v1-amd64
```
