import matplotlib.pyplot as plt
import csv

# File path (update this with your actual file path)
csv_file = "/home/ws/drones_control_main/common/analyze/gps_data.csv"  # Change this to your actual CSV file name

# Data storage
rel_alt, local_z, gps_z_filtered = [], [], []

# Read the CSV file
with open(csv_file, newline='') as file:
    reader = csv.reader(file)
    next(reader)  # Skip the header
    for row in reader:
        rel_alt.append(float(row[0]))
        local_z.append(float(row[1]))
        gps_z_filtered.append(float(row[2]))

# Plot data
plt.figure(figsize=(8, 5))
plt.plot(rel_alt, label="/mavros/global_position/rel_alt", marker='o', linestyle='-')
plt.plot(local_z, label="/mavros/local_position/odom/z", marker='s', linestyle='--')
plt.plot(gps_z_filtered, label="/mavros/global_position/raw/fix/altitude", marker='^', linestyle='-.')

# Labels and legend
plt.xlabel("Sample Index")
plt.ylabel("Altitude (m)")
plt.title("Comparison of Different Altitude Values")
plt.legend()
plt.grid(True)
plt.show()
