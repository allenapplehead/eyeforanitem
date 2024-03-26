import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import yaml

def read_tag_configurations(config_path):
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
    width, height = 0.01, 0.2  # Thickness and length of the line representing the tag
    angle = np.deg2rad(yaw)
    
    # Adjust corner calculation for orientation representation
    corners = np.array([[-height / 2, -width / 2], [height / 2, -width / 2], [height / 2, width / 2], [-height / 2, width / 2]])
    
    # Rotate and translate corners
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    rotated_corners = np.dot(corners, rotation_matrix)
    translated_corners = rotated_corners + np.array([center[1], center[0]])  # Swapping x and y
    
    # Polygon representation
    polygon = patches.Polygon(translated_corners, closed=True, edgecolor='black', facecolor='none')
    ax.add_patch(polygon)
    
    # Adjust arrow for orientation to reflect swapped axes
    arrow_length = 0.1  # meters
    dx = arrow_length * np.sin(angle)  # Swap sin and cos to rotate axes
    dy = arrow_length * np.cos(angle)
    ax.arrow(center[1], center[0], dx, dy, head_width=0.05, head_length=0.1, fc='blue', ec='blue')  # Swapping x and y
    
    # Tag ID placement adjusted
    ax.text(center[1], center[0], f'ID: {tag_id}', fontsize=9, color='black', ha='center')

def visualize_tags(tags):
    plt.figure(figsize=(10, 6))
    ax = plt.gca()
    
    room_size = [5, 3]  # Define room size for visualization
    ax.set_xlim([-1, room_size[1]])  # Adjust limits based on swapped axes
    ax.set_ylim([-1, room_size[0]])
    
    for tag_id, tag_info in tags.items():
        pos = tag_info['position'][:2]
        yaw = tag_info['rotation']
        add_tag(ax, pos, yaw, tag_id)

    ax.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.2, fc='red', ec='red', width=0.02, length_includes_head=True)
    ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.2, fc='green', ec='green', width=0.02, length_includes_head=True)
    
    ax.invert_xaxis()  # Invert x-axis to match visualization
    ax.set_aspect('equal', adjustable='box')

    plt.title('Living Room Tag Configurations')
    plt.xlabel('Y position [m]')  # Labels swapped
    plt.ylabel('X position [m]')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    config_path = '/workspace/robot_ws/src/localizer/config/tags.yaml'  # Update path as necessary
    tags = read_tag_configurations(config_path)
    visualize_tags(tags)
