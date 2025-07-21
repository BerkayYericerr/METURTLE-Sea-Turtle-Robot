# ros2_tcp_node

A ROS 2 Lifecycle Node that receives structured data over a TCP socket and publishes it as a serialized string to a ROS 2 topic.

This node is used to interface with devices like GNSS/IMU sensors over a TCP connection, parse the incoming binary data (after CRC validation), and publish it to a topic for downstream ROS 2 nodes.

---

## Features

- Uses **ROS 2 Lifecycle Node** interface for better control.
- Implements **Reactor pattern** with `epoll` for efficient I/O.
- Parses binary messages (e.g., from DefaSense310) and publishes the structured data.
- CRC validation ensures data integrity before publishing.
- Data is serialized to string format for debugging and logging.

---

##  Folder Structure

