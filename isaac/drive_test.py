#!/usr/bin/env python3
# Prueba de drive: publica /cmd_vel (avance) unos segundos y mide /odom antes/despues.
# Uso: source isaac_env.sh && python3 isaac/drive_test.py [vx] [vy] [wz] [segundos]
import sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

vx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
vy = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
wz = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
dur = float(sys.argv[4]) if len(sys.argv) > 4 else 3.0

rclpy.init()
n = rclpy.create_node("drive_test")
pub = n.create_publisher(Twist, "/cmd_vel", 10)
last = {}
n.create_subscription(Odometry, "/odom", lambda m: last.__setitem__("o", m), 10)

def pose():
    p = last["o"].pose.pose.position
    return p.x, p.y, p.z

# esperar primer /odom
t0 = time.time()
while "o" not in last and time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.1)
if "o" not in last:
    print("NO se recibio /odom en 8s (¿scene_agv arriba? ¿discovery/DDS?)")
    rclpy.shutdown(); sys.exit(1)
print(f"odom inicial: x={pose()[0]:.3f} y={pose()[1]:.3f} z={pose()[2]:.3f}")

# mover
tw = Twist(); tw.linear.x = vx; tw.linear.y = vy; tw.angular.z = wz
t0 = time.time()
while time.time() - t0 < dur:
    pub.publish(tw); rclpy.spin_once(n, timeout_sec=0.02)
print(f"odom moviendo: x={pose()[0]:.3f} y={pose()[1]:.3f} z={pose()[2]:.3f}")

# frenar (importante: el nodo de la escena no tiene watchdog)
stop = Twist()
t0 = time.time()
while time.time() - t0 < 1.0:
    pub.publish(stop); rclpy.spin_once(n, timeout_sec=0.02)
print(f"odom final:   x={pose()[0]:.3f} y={pose()[1]:.3f} z={pose()[2]:.3f}")
print(f"desplazamiento: dx={pose()[0]:.3f} (esperado ~{vx*dur:.2f} si el drive funciona)")
rclpy.shutdown()
