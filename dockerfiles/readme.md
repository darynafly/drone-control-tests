Build dockerfile for pc (server) or raspberry pi with mavros and ROS2 humble:

```
sudo chmod +x build.sh create_container.sh
```

Build docker image on pc
```
./build.sh pc
```

Build docker image on raspberrypi
```
./build.sh rasp
```

Create docker container and start developing

In create_container.sh define folder you want to share between system and docker continer WORKSPACE_DIR="/home/user/ws"

```
./create_container.sh
```