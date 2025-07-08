#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time

class LapTimer(Node):
    def __init__(self):
        super().__init__('lap_timer')
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        # Define el punto de referencia para detectar las vueltas
        self.start_x = 0.0
        self.start_y = 0.0
        self.radius = 2.0  # Radio de detección (margen de error)

        self.started = False
        self.timer_started = False
        self.start_time = None
        self.lap_count = 0
        self.crossed = False

        self.get_logger().info("⏱️ Cronómetro listo. Esperando inicio de la vuelta...")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dist = math.sqrt((x - self.start_x) ** 2 + (y - self.start_y) ** 2)

        if not self.started:
            # Esperar a que el robot se aleje del punto inicial antes de permitir contar vueltas
            if dist > self.radius:
                self.started = True
                self.get_logger().info("🚗 Comenzando vuelta...")
            return

        if not self.timer_started and dist > self.radius:
            # Ahora sí iniciar cronómetro
            self.start_time = time.time()
            self.timer_started = True

        if self.timer_started:
            if dist < self.radius and not self.crossed:
                self.lap_count += 1
                elapsed = time.time() - self.start_time
                mins = int(elapsed // 60)
                secs = int(elapsed % 60)
                self.get_logger().info(f"🏁 Vuelta {self.lap_count} - Tiempo: {mins:02}:{secs:02}")
                self.crossed = True

            elif dist > self.radius:
                self.crossed = False


def main(args=None):
    rclpy.init(args=args)
    node = LapTimer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

