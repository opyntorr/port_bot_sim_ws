#!/usr/bin/env python3
"""
wait_for_tf.py  —  Espera a que una TF (target->source) exista y entonces TERMINA (exit 0).

Pensado para "compuertas" en launch: arrancar un nodo pesado (p.ej. SLAM) solo cuando la
cadena TF que necesita ya está disponible, vía RegisterEventHandler(OnProcessExit(...)).
A diferencia de un TimerAction (reloj de pared), esto respeta el reloj de simulacion
(use_sim_time), así que funciona aunque el RTF sea bajo.

Parametros:
  target (str)       frame padre        (default 'odom')
  source (str)       frame hijo         (default 'base_footprint')
  poll_period (s)    cada cuanto chequea (default 0.5)
  timeout (s)        si la TF no aparece en este tiempo (reloj de sim), sale igual para
                     no colgar el arranque indefinidamente (default 60.0; 0 = sin limite)
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class WaitForTF(Node):
    def __init__(self):
        super().__init__('wait_for_tf')
        self.target = self.declare_parameter('target', 'odom').value
        self.source = self.declare_parameter('source', 'base_footprint').value
        self.poll_period = float(self.declare_parameter('poll_period', 0.5).value)
        self.timeout = float(self.declare_parameter('timeout', 60.0).value)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.t_start = self.get_clock().now()
        self.get_logger().info(
            f"Esperando TF {self.target} -> {self.source} antes de continuar...")
        self.timer = self.create_timer(self.poll_period, self._check)

    def _check(self):
        if self.buf.can_transform(self.target, self.source, Time()):
            self.get_logger().info(
                f"TF {self.target} -> {self.source} disponible. Lanzando lo siguiente.")
            raise SystemExit(0)
        if self.timeout > 0.0:
            elapsed = (self.get_clock().now() - self.t_start).nanoseconds / 1e9
            if elapsed > self.timeout:
                self.get_logger().warn(
                    f"TF {self.target} -> {self.source} no aparecio en {self.timeout:.0f}s "
                    f"(sim). Continuando de todos modos.")
                raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = WaitForTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
