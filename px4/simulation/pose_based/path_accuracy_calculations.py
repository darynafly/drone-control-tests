import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
#import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString
#from mpl_toolkits.mplot3d import Axes3D

def extract_path(path):
    poses = path.poses
    return np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in poses])

def calculate_path_length(path):
    """
    Calculate the total length of a path.
    """
    if len(path) < 2:
        return 0
    return np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))

def path_length_difference(ideal_path_ros, real_path_ros):
    """
    Computes the difference in length of ideal and real paths.
    """
    ideal_path = extract_path(ideal_path_ros)
    real_path = extract_path(real_path_ros)
    
    # Calculate lengths
    ideal_length = calculate_path_length(ideal_path)
    real_length = calculate_path_length(real_path)

    if ideal_length == 0:
        raise ValueError("Ideal path length is zero, cannot compute percentage difference.")

    len_diff = real_length - ideal_length
    return len_diff

def calculate_deviations(ideal_path_ros, real_path_ros):
    """
    Calculate the deviations of the real path points from the ideal path.
    """
    ideal_path = extract_path(ideal_path_ros)
    real_path = extract_path(real_path_ros)
    
    ideal_line = LineString(ideal_path[:, :2])  # Consider only x, y (2D)
    deviations = [Point(p[0], p[1]).distance(ideal_line) for p in real_path]
    
    return np.mean(deviations), np.min(deviations), np.max(deviations), deviations

def calculate_metrics(ideal_path_ros, real_path_ros, threshold=0.5):
    """
    Calculate accuracy, precision, recall, and F1-score.
    """

    # Extract paths as numpy arrays
    ideal_path = extract_path(ideal_path_ros)
    real_path = extract_path(real_path_ros)

    if ideal_path is None or real_path is None or len(ideal_path) == 0 or len(real_path) == 0:
        return None, None, None, None

    # Create a shapely LineString for the ideal path
    ideal_line = LineString(ideal_path[:, :2])  # Only consider x, y for metrics (2D)

    # Initialize counters
    true_positives = 0
    false_positives = 0
    false_negatives = 0

    # Evaluate real path points (True Positives and False Positives)
    for point in real_path:
        shapely_point = Point(point[0], point[1])
        if shapely_point.distance(ideal_line) <= threshold:
            true_positives += 1
        else:
            false_positives += 1

    # Evaluate missed planned path points (False Negatives)
    for point in ideal_path:
        shapely_point = Point(point[0], point[1])
        if all(shapely_point.distance(Point(rp[0], rp[1])) > threshold for rp in real_path):
            false_negatives += 1

    # Accuracy: Proportion of correct predictions (true positives) relative to all points.
    total_points = len(real_path) + len(ideal_path)
    accuracy = (true_positives / total_points * 100) if total_points > 0 else 0

    # Precision: Proportion of true positives out of all predicted positive points.
    precision = (
        true_positives / (true_positives + false_positives) * 100
        if (true_positives + false_positives) > 0
        else 0
    )

    # Recall: Proportion of true positives out of all actual positive points.
    recall = (
        true_positives / (true_positives + false_negatives) * 100
        if (true_positives + false_negatives) > 0
        else 0
    )

    # F1-Score: Harmonic mean of precision and recall.
    f1_score = (
        2 * (precision * recall) / (precision + recall)
        if (precision + recall) > 0
        else 0
    )

    return accuracy, precision, recall, f1_score


def visualize_paths(ideal_path_ros, real_path_ros, threshold=0.5):
    """
    Compare the desired and real paths and display metrics such as deviations and correctness percentage on the plot.
    """
    ideal_path = extract_path(ideal_path_ros)
    real_path = extract_path(real_path_ros)

    mean_deviation, min_deviation, max_deviation, deviations = calculate_deviations(ideal_path_ros, real_path_ros)

    # 3D Plot for path comparison
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Plot the desired path in blue
    ax.plot(ideal_path[:, 0], ideal_path[:, 1], ideal_path[:, 2],
            label='Desired Path', color='blue', linestyle='-', linewidth=2)

    # Plot the real path in red
    ax.plot(real_path[:, 0], real_path[:, 1], real_path[:, 2],
            label='Real Path', color='red', linestyle='--', linewidth=2)

    # Plot the deviation as a color gradient on the real path
    sc = ax.scatter(real_path[:, 0], real_path[:, 1], real_path[:, 2],
                    c=deviations, cmap='viridis', label='Deviation', marker='o')

    # Add color bar
    fig.colorbar(sc, ax=ax, label='Deviation (m)')

    # Set labels and title
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_zlabel('Z (meters)')
    ax.set_title('Path Comparison in 3D')
    ax.legend()

    # Annotate the graph with metrics
    #metrics_text = (
    #    f"Correctness: {correctness_percentage:.2f}%\n"
    #    f"Length Difference: {length_difference:.2f} m\n"
    #    f"Min Deviation: {min_deviation:.2f} m\n"
    #    f"Avg Deviation: {avg_deviation:.2f} m\n"
    #    f"Threshold: {threshold} m"
    #)
    #ax.text2D(0.05, 0.95, metrics_text, transform=ax.transAxes, fontsize=12, verticalalignment='top', bbox=dict(boxstyle="round", alpha=0.5, facecolor='white'))

    # Equal scaling for zoom compatibility
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])
    max_range = max(x_range, y_range, z_range)

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])

    # Enable interaction for zoom and rotation
    plt.subplots_adjust(left=0, right=1, top=1, bottom=0)  # Maximize the plot area
    plt.show()