#!/usr/bin/env python3
# Cuenta publishers/subscribers de los topics clave (sin usar el CLI de ros2, que se cuelga).
import time
import rclpy
rclpy.init()
n = rclpy.create_node("introspect")
t0 = time.time()
while time.time() - t0 < 3.0:      # dejar asentar el discovery
    rclpy.spin_once(n, timeout_sec=0.1)
for t in ["/clock", "/odom", "/tf", "/joint_states", "/cmd_vel"]:
    print(f"{t:14s} pubs={n.count_publishers(t)} subs={n.count_subscribers(t)}")
rclpy.shutdown()
