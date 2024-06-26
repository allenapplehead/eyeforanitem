import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import yaml
import pickle
import sys

def read_tag_configurations(config_path):
    """
    Reads tag configurations from a YAML file.

    Parameters:
    - config_path (str): Path to the configuration YAML file.

    Returns:
    - dict: A dictionary with tag IDs as keys and their properties (position, rotation) as values.
    """
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    
    num_tags = config['num_tags']
    tags = {}
    
    for i in range(num_tags):
        tag_id = config[f'tag_id_{i}']
        tags[tag_id] = {
            'position': config[f'tag_position_{i}'],
            'rotation': config[f'tag_rotation_{i}'][2]  # Assuming rotation about Z-axis for yaw
        }
    
    return tags

def add_tag(ax, center, yaw, tag_id):
    """
    Adds a graphical representation of a tag to the plot.

    Parameters:
    - ax: The matplotlib axis to draw on.
    - center (tuple): The (x, y) center position of the tag.
    - yaw (float): The yaw angle (rotation about the Z-axis) in degrees.
    - tag_id (str): The identifier of the tag.
    """
    width, height = 0.01, 0.2  # Tag dimensions
    angle = np.deg2rad(yaw)
    
    # Calculate the corners of the tag based on its center, size, and rotation
    corners = np.array([[-height / 2, -width / 2], [height / 2, -width / 2], [height / 2, width / 2], [-height / 2, width / 2]])
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    rotated_corners = np.dot(corners, rotation_matrix)
    translated_corners = rotated_corners + np.array([center[1], center[0]])
    
    # Plot the tag as a polygon with an arrow indicating the forward direction
    polygon = patches.Polygon(translated_corners, closed=True, edgecolor='black', facecolor='none')
    ax.add_patch(polygon)
    arrow_length = 0.1
    dx = arrow_length * np.sin(angle)
    dy = arrow_length * np.cos(angle)
    ax.arrow(center[1], center[0], dx, dy, head_width=0.05, head_length=0.1, fc='blue', ec='blue')
    ax.text(center[1], center[0], f'ID: {tag_id}', fontsize=9, color='black', ha='center')

def add_robot_pose(ax, image_name, pose):
    """
    Adds a graphical representation of the robot's pose to the plot.

    Parameters:
    - ax: The matplotlib axis to draw on.
    - image_name (str): The name of the image associated with the robot's pose.
    - pose (tuple): The (x, y, yaw) pose of the robot.
    """
    print("robot pose:", pose)
    x, y, yaw = pose
    angle = yaw

    # Robot represented as a dot
    robot_size = 10  # Size of the dot representing the robot

    # Plot robot pose as a large dot
    ax.plot(y, x, 'ro', markersize=robot_size)  # 'ro' for red dot

    # Add an arrow to indicate the forward direction of the robot
    arrow_length = 0.5  # Length of the arrow representing forward direction
    dx = arrow_length * np.sin(angle)
    dy = arrow_length * np.cos(angle)
    ax.arrow(y, x, dx, dy, head_width=0.05, head_length=0.1, fc='darkred', ec='darkred')

    # Position for the image name text label
    label_distance = arrow_length + 0.15  # Slightly beyond the arrow tip
    label_dx = label_distance * np.sin(angle)
    label_dy = label_distance * np.cos(angle)
    
    # Adding the image name text label in front of the arrow
    ax.text(y + label_dx, x + label_dy, image_name, fontsize=6, color='darkblue', ha='center', va='center')

def visualize_tags(tags, image_name, robot_pose=None):
    """
    Visualizes the tags and optionally the robot's pose on a 2D plot.

    Parameters:
    - tags (dict): Dictionary of tags with their positions and orientations.
    - image_name (str): The name of the image (used for labeling the robot pose).
    - robot_pose (tuple, optional): The (x, y, yaw) pose of the robot.
    """
    plt.figure(figsize=(10, 6))
    ax = plt.gca()
    
    room_size = [5, 3]
    ax.set_xlim([-1, room_size[1]])
    ax.set_ylim([-1, room_size[0]])
    
    for tag_id, tag_info in tags.items():
        pos = tag_info['position'][:2]
        yaw = tag_info['rotation']
        add_tag(ax, pos, yaw, tag_id)
    
    if robot_pose:
        add_robot_pose(ax, image_name, robot_pose)

    ax.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.2, fc='red', ec='red', width=0.02, length_includes_head=True)
    ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.2, fc='green', ec='green', width=0.02, length_includes_head=True)
    
    ax.invert_xaxis()
    ax.set_aspect('equal', adjustable='box')
    
    plt.xlabel('Y position [m]')
    plt.ylabel('X position [m]')
    plt.grid(True)
    if robot_pose:
        plt.legend(['Robot', 'Tags'])
    else:
        plt.legend(['Tags'])
    plt.show()

if __name__ == '__main__':
    config_path = '/workspace/robot_ws/src/localizer/config/tags.yaml'
    tags = read_tag_configurations(config_path)

    if len(sys.argv) != 2:
        print("Image not given, visualizing tags only")
        visualize_tags(tags, None)
        sys.exit(1)
        
    image_name = sys.argv[1]  # Command-line argument for the image name
    stamps_path = '/data/datasets/image_collector/stamps/stamps.pkl'
    
    # Load robot poses from the pkl file
    with open(stamps_path, 'rb') as file:
        stamps = pickle.load(file)
    
    robot_pose = stamps.get(image_name)  # Get the robot pose for the given image name
    
    visualize_tags(tags, image_name, robot_pose)
