# Rescue center and its agents

Remember to: `chmod +x ./packages/rescue_center/src/[node script]`


How to build:
```
dts devel build -f --arch amd64
```

How to run:
```
docker run --name rescue_center --network=host -it --rm -e ROS_MASTER_IP=http://<LAB_SERVER_IP>:11311 duckietown/duckie-rescue-center:v1-amd64
```

For example:
```
docker run --name rescue_center --network=host -it --rm -e ROS_MASTER_IP=http://192.168.1.187:11311 duckietown/duckie-rescue-center:v1-amd64
```
