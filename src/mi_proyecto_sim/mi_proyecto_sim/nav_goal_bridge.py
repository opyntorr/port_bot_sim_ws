#!/usr/bin/env python3
"""
Puente entre RViz (2D Pose Estimate + 2D Goal Pose) y el planificador_rrt existente.

planificador_rrt espera dos TFs en frame `map_dron_origin`:
  - carrito_aruco -> pose inicial (origen del carrito en el mapa cargado)
  - meta_aruco    -> pose meta (destino)

En el pipeline completo, la ALINEACION del mapa cargado con el mundo real la da el
aruco del carrito (publicador_tfs_arucos): un "snapshot" inicial que ancla el mapa
guardado a donde realmente esta el robot, y a partir de ahi SLAM localiza en vivo.

Aqui replicamos ese flujo con RViz, usando la MISMA matematica de alineacion que
publicador_tfs_arucos (math puro, 2D):
  1. "2D Pose Estimate" (/initialpose) = el snapshot del aruco del carrito: indica
     DONDE esta el robot dentro del mapa guardado (su pose cx,cy,ctheta en map_dron_origin).
     Con la pose actual del robot en `map` (de SLAM: mx,my,myaw) calculamos y CONGELAMOS:
        phi = myaw - ctheta
        tx  = mx - (cx*cos(phi) - cy*sin(phi))
        ty  = my - (cx*sin(phi) + cy*cos(phi))
        map -> map_dron_origin = (tx, ty, phi)
     Identico a publicador_tfs_arucos (cx,cy,ctheta vienen del Pose Estimate, no del YAML).
     Asi el mapa cargado (/map_dron) queda superpuesto correctamente al mapa que SLAM
     reconstruye en vivo (/map). El rescaneo (obstaculos dinamicos) sigue intacto.
  2. carrito_aruco sigue al robot EN VIVO (base_footprint) ya alineado -> el RRT
     replanifica desde la pose real mientras avanza (SLAM localiza).
  3. "2D Goal Pose" (/goal_pose) = meta_aruco (destino del RRT).

Mientras NO se reciba un 2D Pose Estimate, la alineacion arranca en identidad (compat con
el flujo de solo-goal anterior). Publica /alignment_ready=True (latched) para destrabar
el control.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class NavGoalBridge(Node):
    def __init__(self):
        super().__init__('nav_goal_bridge')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_bcast = TransformBroadcaster(self)
        # map -> map_dron_origin se publica como TF ESTATICA (no dinamica): /map_dron es un
        # OccupancyGrid latcheado con timestamp viejo, y RViz necesita una TF estatica (valida
        # para cualquier tiempo) para dibujarlo; una TF dinamica a 20Hz hace que RViz descarte
        # el mapa ("timestamp earlier than transform cache"). Se re-emite al cambiar (Pose Estimate).
        self.static_bcast = StaticTransformBroadcaster(self)

        # Alineacion map -> map_dron_origin como (tx, ty, phi). Identidad hasta el
        # primer 2D Pose Estimate; entonces se recalcula y congela.
        self.tx = 0.0
        self.ty = 0.0
        self.phi = 0.0
        self.align_frozen = False

        self.goal_map = None    # (x,y,yaw) del ultimo goal EN frame map (crudo de RViz)
        self.goal_dron = None   # (x,y,yaw) del goal transformado a map_dron_origin

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_cb, 10
        )

        qos_latch = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.alignment_pub = self.create_publisher(Bool, '/alignment_ready', qos_latch)
        self.alignment_pub.publish(Bool(data=True))

        # Pinchazo a /replan_request: el planificador_rrt CANCELA su timer tras el primer
        # plan exitoso, asi que un goal nuevo (o un re-anclaje con Pose Estimate) NO
        # replanifica solo. Hay que pedirselo por este topico (replan_callback -> do_planning,
        # que funciona aunque el timer este cancelado). Se DIFIERE ~0.3s (ver _publish_tfs)
        # para que el meta_aruco nuevo ya este en la TF buffer del planner cuando replanifique.
        self.replan_pub = self.create_publisher(Empty, '/replan_request', 10)
        self._replan_countdown = 0   # ticks de _publish_tfs (0.05s) hasta pinchar replan

        # Publicar la alineacion inicial (identidad) como TF estatica.
        self._publish_align_static()

        self.timer = self.create_timer(0.05, self._publish_tfs)

        self.get_logger().info(
            'nav_goal_bridge listo. "2D Pose Estimate" = pose inicial del carrito en el '
            'mapa (ancla, como el aruco). "2D Goal Pose" = destino.'
        )

    def _publish_align_static(self):
        """Emite map -> map_dron_origin como TF ESTATICA (re-llamar al cambiar la alineacion)."""
        ta = TransformStamped()
        ta.header.stamp = self.get_clock().now().to_msg()
        ta.header.frame_id = 'map'
        ta.child_frame_id = 'map_dron_origin'
        ta.transform.translation.x = float(self.tx)
        ta.transform.translation.y = float(self.ty)
        ta.transform.translation.z = 0.0
        ta.transform.rotation.z = float(math.sin(self.phi * 0.5))
        ta.transform.rotation.w = float(math.cos(self.phi * 0.5))
        self.static_bcast.sendTransform(ta)

    def _request_replan(self):
        """Arma el pinchazo DIFERIDO a /replan_request (~0.3s = 6 ticks de _publish_tfs).
        El retardo deja que el meta_aruco recien actualizado se propague a la TF buffer del
        planificador antes de que replanifique, evitando que planifique con la meta vieja."""
        self._replan_countdown = 6

    def initialpose_cb(self, msg: PoseWithCovarianceStamped):
        """2D Pose Estimate: ancla el mapa cargado a la pose real del robot.

        El usuario indica donde esta el robot DENTRO del mapa guardado (cx,cy,ctheta en
        map_dron_origin). Igual que publicador_tfs_arucos, equiparamos esa pose con la
        pose actual del robot en `map` (de SLAM) para hallar map->map_dron_origin.
        """
        p = msg.pose.pose
        cx = p.position.x
        cy = p.position.y
        ctheta = yaw_from_quat(p.orientation.x, p.orientation.y,
                               p.orientation.z, p.orientation.w)
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            mx = tr.transform.translation.x
            my = tr.transform.translation.y
            mq = tr.transform.rotation
            myaw = yaw_from_quat(mq.x, mq.y, mq.z, mq.w)
        except Exception as e:
            self.get_logger().warn(
                f'No pude leer map->base_footprint para alinear (¿SLAM listo?): {e}')
            return

        self.phi = myaw - ctheta
        self.tx = mx - (cx * math.cos(self.phi) - cy * math.sin(self.phi))
        self.ty = my - (cx * math.sin(self.phi) + cy * math.cos(self.phi))
        self.align_frozen = True
        # Re-emitir la alineacion como TF estatica con el nuevo valor.
        self._publish_align_static()
        # Si ya habia un goal, re-transformarlo con la alineacion nueva y replanificar
        # (el re-anclaje movio meta_aruco respecto al mapa del planner).
        self._recompute_goal_dron()
        if self.goal_map is not None:
            self._request_replan()
        self.get_logger().info(
            f'Alineacion map->map_dron_origin CONGELADA desde 2D Pose Estimate '
            f'(tx={self.tx:.2f}, ty={self.ty:.2f}, phi={math.degrees(self.phi):.1f} deg). '
            f'El carrito se localiza con SLAM desde aqui.')

    def goal_cb(self, msg: PoseStamped):
        # El goal de RViz viene en frame `map` (fixed frame). Guardamos el crudo en map y
        # lo (re)transformamos a map_dron_origin con la alineacion actual.
        gyaw_map = yaw_from_quat(msg.pose.orientation.x, msg.pose.orientation.y,
                                 msg.pose.orientation.z, msg.pose.orientation.w)
        self.goal_map = (msg.pose.position.x, msg.pose.position.y, gyaw_map)
        self._recompute_goal_dron()
        # Cada goal nuevo dispara una replanificacion (diferida ~0.3s). Sin esto, el
        # planner solo planifica el PRIMER goal (cancela su timer) y los siguientes se ignoran.
        self._request_replan()
        if self.goal_dron is not None:
            self.get_logger().info(
                f'Nuevo goal: map=({self.goal_map[0]:.2f},{self.goal_map[1]:.2f}) -> '
                f'map_dron_origin=({self.goal_dron[0]:.2f},{self.goal_dron[1]:.2f})')

    def _recompute_goal_dron(self):
        """Transforma el goal de frame map -> map_dron_origin con la alineacion (tx,ty,phi).
        p_dron = R(-phi) * (p_map - t).  Se llama al recibir goal y al re-anclar (Pose Estimate)."""
        if self.goal_map is None:
            return
        gx_map, gy_map, gyaw_map = self.goal_map
        dx = gx_map - self.tx
        dy = gy_map - self.ty
        c, s = math.cos(-self.phi), math.sin(-self.phi)
        self.goal_dron = (dx * c - dy * s, dx * s + dy * c, gyaw_map - self.phi)

    def _publish_tfs(self):
        now = self.get_clock().now().to_msg()
        # map -> map_dron_origin ya se publica como TF estatica (no aqui).

        # Replan diferido: al armarse (goal/anchor nuevo) cuenta atras y, al expirar
        # (ya propagado el meta_aruco nuevo), pincha /replan_request UNA sola vez.
        if self._replan_countdown > 0:
            self._replan_countdown -= 1
            if self._replan_countdown == 0:
                self.replan_pub.publish(Empty())
                self.get_logger().info('Replan solicitada (/replan_request) por goal/anchor nuevo.')

        # carrito_aruco = pose VIVA del robot en map_dron_origin (SLAM localiza en vivo)
        try:
            trans = self.tf_buffer.lookup_transform(
                'map_dron_origin', 'base_footprint', rclpy.time.Time())
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map_dron_origin'
            t.child_frame_id = 'carrito_aruco'
            t.transform = trans.transform
            self.tf_bcast.sendTransform(t)
        except Exception:
            pass

        # meta_aruco = ultimo 2D Goal Pose, YA transformado a map_dron_origin (ver goal_cb)
        if self.goal_dron is not None:
            gx, gy, gyaw = self.goal_dron
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'map_dron_origin'
            t.child_frame_id = 'meta_aruco'
            t.transform.translation.x = float(gx)
            t.transform.translation.y = float(gy)
            t.transform.translation.z = 0.0
            t.transform.rotation.z = float(math.sin(gyaw * 0.5))
            t.transform.rotation.w = float(math.cos(gyaw * 0.5))
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
