import xml.etree.ElementTree as ET
import os

source_file = '/home/fishros/jixiebi/Dummy-ROS/dummy_ros_sim/urdf/dummy_ros_sim.xacro'
target_file = '/home/fishros/jixiebi/dummy_moveit_ws/dummy-ros2_description/urdf/dummy-ros2.xacro'

tree = ET.parse(source_file)
root = tree.getroot()

# Create new root for ROS 2 structure
new_root = ET.Element("robot", name="dummy-ros2")
new_root.set("xmlns:xacro", "http://www.ros.org/wiki/xacro")

# ROS 2 necessary includes
ET.SubElement(new_root, "xacro:include", filename="$(find dummy-ros2_description)/urdf/materials.xacro")
ET.SubElement(new_root, "xacro:include", filename="$(find dummy-ros2_description)/urdf/dummy-ros2.trans")
ET.SubElement(new_root, "xacro:include", filename="$(find dummy-ros2_description)/urdf/dummy-ros2.gazebo")

# Standard world link for MoveIt
world_link = ET.SubElement(new_root, "link", name="world")
world_joint = ET.SubElement(new_root, "joint", name="world_joint", type="fixed")
ET.SubElement(world_joint, "parent", link="world")
ET.SubElement(world_joint, "child", link="base_link")
ET.SubElement(world_joint, "origin", xyz="0.0 0.0 0.0", rpy="0.0 0.0 0.0")

for child in root:
    # We only bring over links and joints, drop gazebo_ros_control
    if child.tag in ['link', 'joint']:
        # Fix mesh paths to use new package name
        for mesh in child.findall(".//mesh"):
            filename = mesh.get("filename")
            if filename and "dummy_ros_sim" in filename:
                mesh.set("filename", filename.replace("package://dummy_ros_sim/", "package://dummy-ros2_description/"))
                
        # --- Scheme B: Mathematical update + Visual Compensation via Scaling ---
        # 1. Base height (91.25 -> 109)
        if child.tag == 'joint' and child.get('name') == 'joint1':
            origin = child.find('origin')
            origin.set('xyz', '-0.058 0 0.109') 
        elif child.tag == 'link' and child.get('name') == 'base_link':
            # Scale Base Link STL in Z to fill the gap visually: 109 / 91.25 = 1.1945205...
            for geom in child.findall('.//geometry'):
                mesh = geom.find('mesh')
                if mesh is not None:
                    mesh.set('scale', '1 1 1.19452')

        # 2. Forearm length (118.87 -> 115.0)
        elif child.tag == 'joint' and child.get('name') == 'joint5':
            origin = child.find('origin')
            old_z = 0.11887
            new_z = 0.115
            origin.set('xyz', f'0.0109 -0.000096372 {new_z}')
        elif child.tag == 'link' and child.get('name') == 'link4':
            # link4 connects joint4 to joint5, scale its visual to fit new joint5 length
            # Scale = 115 / 118.87 = 0.96744
            for geom in child.findall('.//geometry'):
                mesh = geom.find('mesh')
                if mesh is not None:
                    mesh.set('scale', '1 1 0.96744')

        # 3. Wrist length (70.746 -> 72.0)
        elif child.tag == 'joint' and child.get('name') == 'joint6':
            origin = child.find('origin')
            origin.set('xyz', '-0.072 0 0.010807')
        elif child.tag == 'link' and child.get('name') == 'link5':
            # Scale = 72 / 70.746 = 1.01772
            for geom in child.findall('.//geometry'):
                mesh = geom.find('mesh')
                if mesh is not None:
                    mesh.set('scale', '1.01772 1 1')

        new_root.append(child)

ET.indent(new_root, space="  ", level=0)
tree = ET.ElementTree(new_root)

# Save the beautifully adapted xacro
with open(target_file, 'wb') as f:
    f.write(b'<?xml version="1.0" ?>\n')
    tree.write(f, encoding='utf-8')

print("URDF migration with visual compensation successfully completed.")
