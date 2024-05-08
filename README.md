# ROS2_basics_pt1

## Core Concepts
- ROS2 Installation, Workspace, Packages, …
- ROS2 Nodes
- ROS2 Topics
- ROS2 Services
- ROS2 Custom Messages
- ROS2 Parameters
- ROS2 Launch Files

## Commands

### source python environment
`source ~/ros2_ws/install/setup.bash`

### source cpp environment
`source ~/ros2_ws/src/install/setup.bash`

### run ros node
`ros2 run <package_name> <node_name>`

### run ros node with new name
`ros2 run <package_name> <node_name> --ros-args -r __node:=<new_node_name>`

### list active ros nodes
`ros2 node list`

### get active rose node info
`ros2 node info <active_node_name>`

### build all packages:
`colcon build`

### build specific package:
`colcon build --packages-select <package_name>`

### build with symlink (we don't have to re-build everytime we make modifications)
- Note: This is only useful for python packages since there's no compilation needed as opposed to cpp. Also make sure python node file is an executable so that it can be found by symlink.

`colcon build --packages-select <package_name> --symlink-install`

### make a file an executable
`chmod +x <filename>`

### view ros node graph
`rqt_graph`

### turtlesim demo
- install: 

    `sudo apt install ros-humble-turtlesim`
- launch turtlesim window: 

    `ros2 run turtlesim turtlesim_node`
- move turtle: 

    `ros2 run turtlesim turtle_teleop_key`

