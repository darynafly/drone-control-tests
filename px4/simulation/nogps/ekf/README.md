git clone -b humble-devel https://github.com/cra-ros-pkg/robot_localization.git

Replace ekf.yaml in param folder with the one provided here
Republish relative altitude as odom1, it is needed for ekf

Disable GPS for px4
sensor_gps_sim stop

gazebo_pose_ros - ground truth from gazebo 
python3 repub_alt.py & python3 repub_pose.py & python3 gazebo_pose_ros.py

colcon build --packages-select robot_localization --parallel-workers 20 && ros2 launch robot_localization ekf.launch.py

