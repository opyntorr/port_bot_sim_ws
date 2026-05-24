#!/usr/bin/env python3
"""
Puente entre RViz (2D Goal Pose) y el planificador_rrt existente.

planificador_rrt espera dos TFs en frame `map_dron_origin`:
  - carrito_aruco -> pose inicial (origen del carrito)
  - meta_aruco    -> pose meta (cubo / destino)

En el flujo original ambas TFs venian de publicador_tfs_arucos leyendo
arucos.yaml. Para la navegacion manual desde RViz necesitamos generarlas
en vivo: este nodo publica
  - TF estatica  map -> map_dron_origin               (identidad)
  - TF dinamica  map_dron_origin -> carrito_aruco     (pose actual del robot,
                                                       derivada de base_footprint)
  - TF dinamica  map_dron_origin -> meta_aruco        (ultimo /goal_pose
                                                       recibido desde RViz)
Tambien publica /alignment_ready=True (latched) para destrabar al
control_trayectoria, que normalmente espera a publicador_tfs_arucos.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster


class NavGoalBridge(Node):
    def __init__(self):
        super().__init__('nav_goal_bridge')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_bcast = TransformBroadcaster(self)
        self.static_tf_bcast = StaticTransformBroadcaster(self)

        self._publish_static_map_origin()

        self.goal_pose = None
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10
        )

        qos_latch = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.alignment_pub = self.create_publisher(Bool, '/alignment_ready', qos_latch)
        self.alignment_pub.publish(Bool(data=True))

        self.timer = self.create_timer(0.05, self._publish_dynamic_tfs)

        self.get_logger().info(
            'nav_goal_bridge listo. Usa "2D Goal Pose" en RViz para fijar destino.'
        )

    def _publish_static_map_origin(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'map_dron_origin'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf_bcast.sendTransform(t)

    def goal_cb(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(
            f'Nuevo goal en frame={msg.header.frame_id}: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def _publish_dynamic_tfs(self):
        now = self.get_clock().now().to_msg()

        try:
            trans = self.tf_buffer.lookup_transform(
                'map_dron_origin', 'base_footprint', rclpy.time.Time()
            )
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map_dron_origin'
            t.child_frame_id = 'carrito_aruco'
            t.transform = trans.transform
            self.tf_bcast.sendTransform(t)
        except Exception:
            pass

        if self.goal_pose is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map_dron_origin'
            t.child_frame_id = 'meta_aruco'
            t.transform.translation.x = self.goal_pose.pose.position.x
            t.transform.translation.y = self.goal_pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = self.goal_pose.pose.orientation
            self.tf_bcast.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
