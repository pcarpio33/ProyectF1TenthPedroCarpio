import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class TeleopEmergency(Node):
    def __init__(self):
        super().__init__('teleop_emergency')

        # Subscripciones
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # Publicador
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Variables internas
        self.latest_cmd = Twist()
        self.obstacle_distance = float('inf')

    def cmd_callback(self, msg):
        self.latest_cmd = msg

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()  # ✅ necesario para que el simulador lo acepte
        drive_msg.drive.steering_angle = msg.angular.z

        # Lógica de frenado de emergencia
        if self.obstacle_distance <= 0.5:
            drive_msg.drive.speed = 0.0
            self.get_logger().warn(f"🛑 Paro de emergencia: obstáculo a {self.obstacle_distance:.2f} m")
        elif self.obstacle_distance <= 1.5:
            drive_msg.drive.speed = min(msg.linear.x, 0.8)
            self.get_logger().info(f"⚠️ Reducción de velocidad: obstáculo a {self.obstacle_distance:.2f} m")
        else:
            drive_msg.drive.speed = msg.linear.x

        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, msg):
        center = len(msg.ranges) // 2
        front_window = msg.ranges[center - 15 : center + 15]
        valid = [r for r in front_window if 0.05 < r < 20.0]

        if valid:
            new_distance = round(min(valid), 2)
        else:
            new_distance = float('inf')

        if not hasattr(self, 'last_distance'):
            self.last_distance = new_distance
            self.get_logger().info(f"📏 Distancia frontal inicial: {new_distance:.2f} m")
        elif abs(new_distance - self.last_distance) > 0.1:
            self.last_distance = new_distance
            self.get_logger().info(f"📏 Nueva distancia frontal: {new_distance:.2f} m")

        self.obstacle_distance = new_distance

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        new_x = round(pos.x, 2)
        new_y = round(pos.y, 2)

        if not hasattr(self, 'last_position'):
            self.last_position = (new_x, new_y)
            self.get_logger().info(f'📍 Posición inicial -> x = {new_x}, y = {new_y}')
        elif (new_x, new_y) != self.last_position:
            self.last_position = (new_x, new_y)
            self.get_logger().info(f'📍 Posición actual -> x = {new_x}, y = {new_y}')

        velocity = round(msg.twist.twist.linear.x, 2)
        if not hasattr(self, 'last_velocity'):
            self.last_velocity = velocity
            self.get_logger().info(f'🚗 Velocidad inicial: {velocity:.2f} m/s')
        elif abs(velocity - self.last_velocity) > 0.1:
            self.last_velocity = velocity
            self.get_logger().info(f'🚗 Velocidad actual: {velocity:.2f} m/s')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopEmergency()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






