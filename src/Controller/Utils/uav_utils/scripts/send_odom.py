#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import tf_transformations as tfs
from nav_msgs.msg import Odometry



if __name__ == "__main__":
    # Initialize the ROS 2 node
    rclpy.init()
    node = rclpy.create_node("odom_sender")

    msg = Odometry()
    msg.header.stamp = node.get_clock().now().to_msg()  
    msg.header.frame_id = "world"

    q = tfs.quaternion_from_euler(0, 0, 0, "rzyx")
    msg.pose.pose.position.x = 0.0
    msg.pose.pose.position.y = 0.0
    msg.pose.pose.position.z = 0.0
    msg.twist.twist.linear.x = 0.0
    msg.twist.twist.linear.y = 0.0
    msg.twist.twist.linear.z = 0.0
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]

    print(msg)

    pub = node.create_publisher(Odometry, "odom", 10)

    counter = 0
    rate = node.create_rate(1)  # 1 Hz

    while rclpy.ok():
        counter += 1
        # Update the timestamp
        msg.header.stamp = node.get_clock().now().to_msg()  
        pub.publish(msg)
        node.get_logger().info(f"Send {counter:3d} msg(s).")
        rate.sleep()

    # Shut down the node
    node.destroy_node()
    rclpy.shutdown()
