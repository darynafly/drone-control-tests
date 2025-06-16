### Analyze 2 localizations

First align two pathes by yaw, set correct degrees as commented in the bottom of the scripts:

```
python3 repub_odom_yaw_shift.py 
python3 repub_imu_odom_yaw_shift.py 
```

Then run IMU localization (or any other) 
```
drones_control/px4/raspi_onboard/pose_estimation/simple# python3 imu_to_pose.py
```

Run analyzation, in the script set correct waypoints (self.waypoints)

```
python3 analyze_path.py
```