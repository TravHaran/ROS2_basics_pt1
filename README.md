# ROS2_basics_pt1

## Core Concepts
- ROS2 Installation, Workspace, Packages, …
- ROS2 Nodes
    - Subprograms in your application, responsible for only one thing
    - Combined into a graph
    - Communicate with each other through topics, services, and parameters
    - Benefits:
        - Reduce code complexity
        - Fault tolerance
        - Can be written in Python, C++, etc.
    - To implement nodes in your ROS2 application:
        - Before creating your first node you need to: 
            - create a ROS2 workspace and source it
            - create a (python/cpp) package
        - Then, you write your node using the appropriate ROS2 client library: rclpy for Python, and rclcpp for Cpp. Both libraries will provide the same core functionalities.
        - After writing the node, you compile it, and you re-source your environment in order to use it. Nodes are compiled (only for Cpp), and installed (for both Python and Cpp), inside the install/ folder of your ROS2 workspace. You can directly execute them from here, or by using the command line tool `ros2 run <package> <executable>`.
- ROS2 Topics
    - A topic is a named bus over which nodes exchange messages
    - Unidirectional data stream (publisher/subscriber)
    - Anonymous
    - A topic has a message type
    - Can be written in Python, C++, .. directly inside ROS nodes
    - A node can have many publishers/subscriber for many different topics
    - To implement topics in your ROS2 application:
        - First create a node (or start from an existing one), then inside your node you can create any number of publishers/subscribers
        - A publisher and subscriber must publish/subscribe to the same topic name, and use the same data type. Those are the 2 conditions for successful topic communication
        - Then once you’ve added some publishers/subscribers in your nodes, just launch your nodes, and the communication starts! You can debug them using the “ros2” command line tool, as well as rqt.
- ROS2 Services
    - is a client/server system
    - can be synchronous or asynchronous
    - one message type for reques, one message type for response
    - can be written in python, c++, ... directly inside ROS nodes
    - a service server can only exist once, but can have many clients
- ROS2 Custom Messages
- ROS2 Parameters
- ROS2 Launch Files

## Python Node template
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
 
class MyCustomNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("node_name") # MODIFY NAME
 
 
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
```
## Cpp Node template
```
#include "rclcpp/rclcpp.hpp"
 
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("node_name") // MODIFY NAME
    {
    }
 
private:
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

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

### get specific node info
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

### list ros topics
`ros2 topic list`

### print topic output
`ros2 topic echo <topic_name>`

### get specific topic info
`ros2 topic info <topic_name>`

### get publishing frequency of topic
`ros2 topic hz <topic_name>`

### get topic bandwidth
`ros2 topic bw <topic_name>`

### publish topic from command line
`ros2 topic pub -r <dead_queue_size> <topic_name> <data_type> <message>`
- ex: 
`ros2 topic pub -r 10 /robot_news example_interfaces/msg/String "{data: 'hello from terminal'}" 
`
### get interface info
`ros2 interface show <interface_name>`

### list services
`ros2 service list`

### make service call from command line
`ros2 service call <service_name> <service_datatype> <request>`
- ex: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"`

you can also remap a service name with --ros-args -r <service_name>:=<new_name>
- ex: `ros2 run my_cpp_pkg add_two_ints_server --ros-args -r add_two_ints:=new_name'
