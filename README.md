# Controlling the Drone with MAVROS

This repository contains code and instructions for controlling a drone using **FMU** (Flight Management Unit) and **MAVROS** (MAVLink for ROS2). 

### **MAVROS Folder**

The **MAVROS** folder contains the MAVROS node and code used to control the drone via **MAVLink** (the communication protocol for drones). This integration runs as a bridge between the drone's FMU and your ROS2 environment, allowing to send commands and receive telemetry using standard ROS messages and services.

## Run Simulation

### Start the simulation (On Ubuntu Laptop):

make px4_sitl gz_x500

## How to Control the Drone

### **1. Install MAVROS**

**ROS2** must be installed and properly set up, and then install the **MAVROS** package.

#### For ROS2 Humble:

```bash
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

#### Install GeographicLib datasets:

```bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

For more information, refer to the [MAVROS GitHub README](https://github.com/mavlink/mavros/blob/ros2/mavros/README.md).

### **2. Running MAVROS**

To start the MAVROS node, use the following command depending on your setup:

- **For SITL (Software In The Loop) simulation:**

  ```bash
  ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
  ```

- **For real robot hardware (Pixhawk):**

  ```bash
  ros2 launch mavros px4.launch fcu_url:=/dev/ttyPixhawk
  ```

### **3. Setting Modes**

You can change the flight modes using the following service calls:

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "MANUAL"}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "STABILIZED"}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "AUTO.TAKEOFF"}'
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "AUTO.LAND"}'
```

### **4. Arming and Disarming**

To arm and disarm the drone:

- **Arm:**

  ```bash
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
  ```

- **Disarm:**

  ```bash
  ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: false}'
  ```

### **5. Monitoring Drone State**

To monitor the drone’s state (armed, mode, etc.), use:

```bash
ros2 topic echo /mavros/state
```

### **6. Monitoring Battery and Position**

- **Battery Status:**

  ```bash
  ros2 topic echo /mavros/battery
  ```

- **Global Position:**

  ```bash
  ros2 topic echo /mavros/global_position/global
  ```

### **7. Takeoff and Landing Commands**

- **Takeoff Command**:

  ```bash
  ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{altitude: 4.5, latitude: 0.0, longitude: 0.0, min_pitch: 0.0, yaw: 0.0}'
  ```

- **Land Command**:

  ```bash
  ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL '{altitude: 0.0, latitude: 0.0, longitude: 0.0, min_pitch: 0.0, yaw: 0.0}'
  ```

### **8. Moving the Drone**

To move the drone, you can publish velocity commands using the following:

- **Moving Backward**:

  ```bash
  ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

- **Moving Forward (Continuous)**:

  ```bash
  ros2 topic pub -r 10 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

- **Stop the Drone**:

  ```bash
  ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

### **9. Offboard Control** 

To set the flight mode to **GUIDED** and arm the drone, use these commands:

- **Set Flight Mode to GUIDED**:

  ```bash
  ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "GUIDED"}'
  ```

- **Takeoff Command**:

  ```bash
  ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{altitude: 4.5, latitude: 0.0, longitude: 0.0, min_pitch: 0.0, yaw: 0.0}'
  ```

### **10. Kill the Drone (Emergency Stop)**

In case of an emergency, you can command the drone to stop its current operation:

```bash
ros2 service call /mavros/mavros/command mavros_msgs/srv/CommandLong "{broadcast: false, command: 400, confirmation: 0, param1: 0.0, param2: 21196.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

## About PX4

PX4 is an open-source flight control software stack for drones, offering high-performance, safety-tested, and configurable features for UAVs.

For more information on PX4, visit the [PX4 Documentation](https://docs.px4.io/main/en/getting_started/px4_basic_concepts.html).

## Additional Resources

- [MAVLink](https://mavlink.io/en/) – The communication protocol for drones, used by flight controllers, ground control stations, and peripherals.
- [MAVROS GitHub](https://github.com/mavlink/mavros) – ROS2 node for MAVLink protocol.
- [PX4 Autopilot](http://px4.io/) – Flight Controller with support for most vehicle types and tested MAVROS support.
- [ArduPilot](http://ardupilot.com/) – Autopilot software tested with APM:Plane and other vehicle types.
- [QGroundControl](http://qgroundcontrol.org/) – Ground Control Station for MAVLink autopilots with cross-platform support.

---
