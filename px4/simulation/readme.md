### Run drone without GPS

Disable GPS in simulation console:
```
sensor_gps_sim stop
```

Then IMU localization:
```
python3 pose_estimation/simple/imu_to_pose.py
```

Republish poses for pixhawk ekf:
```
python3 nogps/repub_pose.py
python3 nogps/gazebo_pose_ros.py
```

Run follow waypoints:
```
python3 pose_based/2_simple_planner.py
```

If the drone is not arming nor taking off try to arm in QGroundControl.