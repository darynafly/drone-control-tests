import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read the CSV data
df = pd.read_csv("/home/ws/drones_control/px4/simulation/pose_estimation/imu_odometry_data.csv")

# Extract relevant columns for plotting
x = df['x']
y = df['y']
z = df['z']
yaw = df['yaw']

# Create a 3D plot for the position (x, y, z)
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot the trajectory in 3D space
ax.plot(x, y, z, label='Trajectory', color='b')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Trajectory (Pose)')

# Optionally, plot yaw as a separate 2D plot
fig_yaw = plt.figure(figsize=(10, 4))
plt.plot(yaw, label='Yaw', color='r')
plt.xlabel('Index')
plt.ylabel('Yaw')
plt.title('Yaw over Time')
plt.legend()

plt.show()
