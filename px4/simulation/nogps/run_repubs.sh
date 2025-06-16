#!/bin/bash

# Start the Python scripts in the background
python3 repub_alt.py &
PID1=$! &

python3 repub_pose.py &
PID2=$! &
 
python3 gazebo_pose_ros.py &
PID3=$!

# Trap Ctrl+C (SIGINT) to kill all processes
trap "echo 'Terminating...'; kill $PID1 $PID2 $PID3; exit" SIGINT

# Wait for all processes to complete
wait
