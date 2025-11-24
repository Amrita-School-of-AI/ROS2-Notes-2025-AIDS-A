# ROS2 Notes

### Executables from packages (built with colcon)

1.  Install a well known pacakge, e.g., turtlesim

```shell
sudo apt update
sudo apt install ros-jazzy-turtlesim
```

2.  See list of packages

```shell
ros2 pkg list
```

3.  See list of executables

```shell
ros2 pkg executables
```

4.  See list of executables in a specific package

```shell
ros2 pkg executables turtlesim
```

5. Where is turtlesim package?

```shell
cd $(ros2 pkg prefix turtlesim)
code .
Ctrl+P -> trurtlesim
```

6. Run an executable from a package

```shell
ros2 run <package_name> <executable_name>
```

e.g:

```shell
ros2 run turtlesim turtlesim_node
```

### Nodes

![alt text](images/communication.png)

1. See ros2 node help

```shell
ros2 node --help
```

2. See list of current nodes

```shell
# run a node first
ros2 run <package_name> <executable_name>
ros2 node list
```

3. See info about a node

```shell
ros2 node info /<node_name>
```

### Communication (Topics)

![alt text](images/topic.png)

1. First, run a node that publishes topics

```shell
ros2 run turtlesim turtlesim_node
```

2. See ros2 topic help

```shell
ros2 topic --help
```

3. See list of current topics

```shell
ros2 topic list

# see topic details with types
ros2 topic list -t

# see topic details with publishers and subscribers
ros2 topic list -v
```

4. See a graphical representation of topics

```shell
rqt_graph
```

5. Echo messages from a topic

```shell
ros2 topic echo /<topic_name>

# e.g., (check with turtle_teleop_key node after running this command)
ros2 topic echo /turtle1/cmd_vel
```

6. See info about a topic

```shell
ros2 topic info /<topic_name>
```

7. See info about a `topic type` (`type` is the message type used by the topic)

```shell
ros2 topic type /<topic_name>

# Show the message definition
ros2  interface show <type_name>

# e.g.,
ros2 interface show geometry_msgs/msg/Twist
```

8. Publish messages to a topic

```shell
ros2 topic pub /<topic_name> <type_name> '{<field1>: <value1>, <field2>: <value2>, ...}'

# e.g., (move turtle1)
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}'
```

9 . Publish messages to a topic at a specific rate

```shell
ros2 topic pub -r <rate_in_Hz> /<topic_name> <type_name> '{<field1>: <value1>, <field2>: <value2>, ...}'

# e.g., (move turtle1 at 1 Hz)
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}'
```

10. Check the pose of the turtle by echoing the `/turtle1/pose` topic

```shell
ros2 topic echo /turtle1/pose

# show average rate of messages being published to a topic
ros2 topic hz /turtle1/pose
```
