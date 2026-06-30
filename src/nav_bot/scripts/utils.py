import os
import yaml
import tempfile

def create_namespaced_bridge_yaml(base_yaml_path, namespace):
    """
    Reads a standard bridge.yaml and prefixes all topics with the robot namespace.
    This ensures bot1/scan stays separated from bot2/scan in the Gazebo Bridge.
    """
    if not os.path.exists(base_yaml_path):
        print(f"Error: Bridge template not found at {base_yaml_path}")
        return None

    with open(base_yaml_path, 'r') as f:
        bridges = yaml.safe_load(f)

    # Ensure namespace ends with a slash for topic prefixing
    namespace_prefix = namespace.strip('/') + '/'

    namespaced_bridges = []
    for bridge in bridges:
        # We don't namespace the global clock topic
        if bridge['ros_topic_name'] == 'clock' or bridge['gz_topic_name'] == 'clock':
            namespaced_bridges.append(bridge)
            continue

        # Prefix the ROS topic
        bridge['ros_topic_name'] = f"{namespace_prefix}{bridge['ros_topic_name'].lstrip('/')}"
        
        # Prefix the Gazebo topic (Harmonic uses /model/robot_name/...)
        # We assume the template uses relative names like 'scan'
        bridge['gz_topic_name'] = f"/model/{namespace}/{bridge['gz_topic_name'].lstrip('/')}"
        
        namespaced_bridges.append(bridge)

    # Save to a temporary file in /tmp so Gazebo can read it
    output_path = os.path.join(tempfile.gettempdir(), f"{namespace}_bridge.yaml")
    with open(output_path, 'w') as f:
        yaml.dump(namespaced_bridges, f)

    return output_path


def generate_rviz_config(robot_name, base_config_path):
    """
    Creates a temporary RViz file where the Fixed Frame and Robot Description 
    topic are automatically set to the robot's specific namespace.
    """
    if not os.path.exists(base_config_path):
        print(f"Error: RViz template not found at {base_config_path}")
        return None

    with open(base_config_path, 'r') as f:
        config_text = f.read()

    # Arshadlab Technique: Replace placeholders with the actual robot name
    # In your base_rviz file, you should use <ROBOT_NAME> where you want the name to appear
    config_text = config_text.replace('<ROBOT_NAME>', robot_name)
    
    # Also handle the common case where 'map' needs to be 'bot1/map'
    config_text = config_text.replace('Fixed Frame: map', f'Fixed Frame: {robot_name}/map')

    output_path = os.path.join(tempfile.gettempdir(), f"{robot_name}_nav.rviz")
    with open(output_path, 'w') as f:
        f.write(config_text)

    return output_path


def patch_robot_description(robot_description_string, namespace):
    """
    As a safety net, this function takes the URDF/Xacro string and 
    replaces any remaining hardcoded 'odom' or 'base_link' strings 
    with namespaced versions before spawning.
    """
    # Replace common frame names in the XML string
    replacements = {
        '<remapping>odom:=odom</remapping>': f'<remapping>odom:={namespace}/odom</remapping>',
        '<frame_id>odom</frame_id>': f'<frame_id>{namespace}/odom</frame_id>',
        '<child_frame_id>base_link</child_frame_id>': f'<child_frame_id>{namespace}/base_link</child_frame_id>',
    }

    for old, new in replacements.items():
        robot_description_string = robot_description_string.replace(old, new)

    return robot_description_string