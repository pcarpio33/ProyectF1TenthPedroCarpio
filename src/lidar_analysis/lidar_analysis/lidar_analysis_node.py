import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LiDARAnalysisNode(Node):
    def __init__(self):
        super().__init__('lidar_analysis_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.previous_time = None
        self.received_once = False

    def lidar_callback(self, msg: LaserScan):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Resolución angular en grados
        angle_increment_deg = math.degrees(msg.angle_increment)
        print(f"Resolución angular del LiDAR: {angle_increment_deg:.3f} grados")

        # Frecuencia de publicación
        if self.previous_time is not None:
            period = current_time - self.previous_time
            frequency = 1.0 / period
            print(f"Frecuencia del LiDAR: {frequency:.3f} Hz")
        else:
            self.previous_time = current_time
            return  # esperamos un mensaje más para poder calcular la frecuencia

        #  División en front y rear usando ángulos reales
        front_ranges = []
        rear_ranges = []

        angle = msg.angle_min  # radianes

        for i, r in enumerate(msg.ranges):
            angle_deg = math.degrees(angle) % 360  # lo llevamos a 0-360°
            if 0 <= angle_deg <= 180:
                front_ranges.append(r)
            else:
                rear_ranges.append(r)
            angle += msg.angle_increment

        print(f"Cantidad de mediciones frontales (0° a 180°): {len(front_ranges)}")
        print(f"Cantidad de mediciones traseras (180° a 360°): {len(rear_ranges)}")

        print(f"Primeros valores frontales: {front_ranges[:5]}")
        print(f"Primeros valores traseros: {rear_ranges[:5]}")

        # Apagamos el nodo luego de imprimir todo
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LiDARAnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


