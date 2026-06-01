#!/usr/bin/env python3
# encoding: utf-8
"""
SunriseRobotLib.py (MOCK PARA SIMULACIÓN GAZEBO)
Esta librería emula la placa de hardware física del Yahboom Monster AMR.
En lugar de comunicarse por I2C (smbus) con hardware real, lee y escribe
hacia Gazebo usando tópicos estándar de ROS 2.

Al colocar este archivo en tu paquete, tu script original `Mcnamu_driver_PID.py`
lo importará, creyendo que está en el hardware real, y no tendrás que cambiarle
ni una sola línea de código.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class GazeboBridgeNode(Node):
    """Nodo oculto que actúa como puente entre el hardware falso y Gazebo"""
    def __init__(self):
        super().__init__('sunrise_robot_gazebo_bridge')
        
        # --- Configuración Simulador ---
        # Asumiremos un controlador de esfuerzo tipo JointGroupEffortController
        self.joint_names = ['front_left_wheel_joint', 'back_left_wheel_joint', 'front_right_wheel_joint', 'back_right_wheel_joint']
        self.max_torque = 2.5  # Restablecido a 2.5 Nm ya que el URDF ahora tiene amortiguación física
        
        # --- Estado Hardware Simulado ---
        # Ticks del encoder [M1(FL), M2(RL), M3(FR), M4(RR)]
        self.encoder_ticks = [0, 0, 0, 0] 
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.motion = (0.0, 0.0, 0.0)
        
        # --- Suscriptores (Gazebo -> "Hardware") ---
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        # Suscripciones opcionales para la telemetría (se asumen tópicos comunes de Gazebo)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # --- Publicadores ("Hardware" -> Gazebo) ---
        self.pub_effort = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
    def joint_cb(self, msg: JointState):
        # Mapear RADIANES (Gazebo) -> TICKS DE ENCODER (Hardware)
        # El motor original tiene 1440 ticks por revolución (2*pi radianes)
        rad_to_ticks = 1440.0 / (2.0 * math.pi)
        
        for i, j_name in enumerate(self.joint_names):
            if j_name in msg.name:
                idx = msg.name.index(j_name)
                rad = msg.position[idx]
                self.encoder_ticks[i] = int(rad * rad_to_ticks)
                
    def imu_cb(self, msg: Imu):
        self.accel = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self.gyro = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        
    def odom_cb(self, msg: Odometry):
        self.motion = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)
        
    def set_pwm(self, fl, rl, fr, rr):
        # Mapear PWM de hardware [-100 a 100] -> TORQUE para Gazebo [-max_torque a max_torque]
        # OJO: set_motor recibe (FL, RL, FR, RR). 
        # Pero el Float64MultiArray de Gazebo debe ir en el orden configurado en controllers.yaml
        # Asumiremos el orden clásico: [FL, FR, RL, RR] para el arreglo de comandos.
        efforts = [
            (fl / 100.0) * self.max_torque, # FL
            (fr / 100.0) * self.max_torque, # FR
            (rl / 100.0) * self.max_torque, # RL
            (rr / 100.0) * self.max_torque  # RR
        ]
        msg = Float64MultiArray()
        msg.data = efforts
        self.pub_effort.publish(msg)


class SunriseRobot:
    """Clase principal que simula la API original de SunriseRobotLib"""
    
    def __init__(self):
        # Arrancamos nuestro nodo "puente" y lo ponemos a girar en un hilo de fondo
        # Nota: rclpy.init() ya debió ser llamado por el script principal
        self.bridge = GazeboBridgeNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        
    def set_car_type(self, type_num):
        pass # Ignorado en simulación
        
    def create_receive_threading(self):
        pass # El hilo puente ya fue creado en el constructor
        
    def get_motor_encoder(self):
        # El hardware original devuelve la tupla (M1, M2, M3, M4) que corresponde a (FL, RL, FR, RR)
        return tuple(self.bridge.encoder_ticks)
        
    def set_motor(self, m1, m2, m3, m4):
        # El hardware original asume que m1=FL, m2=RL, m3=FR, m4=RR
        self.bridge.set_pwm(m1, m2, m3, m4)
        
    def get_battery_voltage(self):
        return 12.0 # Batería siempre llena en simulación :)
        
    def get_accelerometer_data(self):
        return self.bridge.accel
        
    def get_gyroscope_data(self):
        return self.bridge.gyro
        
    def get_motion_data(self):
        return self.bridge.motion
