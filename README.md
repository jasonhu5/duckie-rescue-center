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

For reference: [unicode characters used for map visualization](https://www.compart.com/en/unicode/block/U+2500)


# Visualization
  docker run -it --rm --net=host --env="DISPLAY" -e ROS_MASTER=duckietown9 -e ROS_MASTER_IP=192.168.1.187 -e DUCKIETOWN_WORLD_FORK=jasonhu5 -e MAP_NAME=ethz_amod_lab_k31 duckietown/dt-autolab-rviz
