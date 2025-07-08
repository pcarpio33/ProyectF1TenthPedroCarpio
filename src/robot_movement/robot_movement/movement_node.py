import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import math


class NavegadorRectangular(Node):
    def __init__(self):
        super().__init__('navegador_rectangular')
        
        # Publicador de comandos de movimiento tipo Ackermann
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # Suscripción al tópico de odometría del robot
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.procesar_odometria, 10)

        # Lista de puntos objetivo que definen la trayectoria rectangular
        self.puntos = [
            (9.0, 0.0),
            (9.6, 8.1),
            (-13.01, 8.75),
            (-13.67, 0.58),
            (9.0, 0.0)  # Vuelve al punto inicial
        ]

        self.etapa = 0              # Índice del punto actual a alcanzar
        self.estado = 'AVANCE'      # Estado del robot: 'AVANCE' o 'GIRO'
        self.tolerancia = 0.5       # Margen para considerar que se ha llegado al punto
        self.yaw_actual = None      # Orientación actual
        self.yaw_inicial = None     # Orientación antes del giro
        self.angulo_giro = math.radians(90)  # Ángulo objetivo de cada giro

    def procesar_odometria(self, msg):
        # Extrae la posición y orientación del mensaje de odometría
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.posicion = (pos.x, pos.y)
        self.yaw_actual = self.obtener_yaw(ori)

        # Decide si avanzar o girar en función del estado actual
        if self.estado == 'AVANCE':
            self.mover_hacia_objetivo()
        elif self.estado == 'GIRO':
            self.realizar_giro()

    def mover_hacia_objetivo(self):
        # Calcula la distancia al siguiente punto objetivo
        destino = self.puntos[self.etapa]
        dx = destino[0] - self.posicion[0]
        dy = destino[1] - self.posicion[1]
        distancia = math.hypot(dx, dy)

        # Publica un comando para avanzar en línea recta
        self.publicar_comando(1.5, 0.0)
        self.get_logger().info(f'Avanzando a {destino} | Distancia: {distancia:.2f} m')

        # Cambia al estado de giro si se llegó al punto objetivo
        if distancia <= self.tolerancia:
            self.yaw_inicial = self.yaw_actual
            self.estado = 'GIRO'

    def realizar_giro(self):
        # Calcula cuánto ha girado desde el inicio del giro
        diferencia = self.ajustar_angulo(self.yaw_actual - self.yaw_inicial)
        
        # Publica un comando de giro suave
        self.publicar_comando(0.5, 0.275)
        self.get_logger().info(f'Girando... Ángulo actual: {math.degrees(diferencia):.2f}°')

        # Si se completó el giro deseado, pasa al siguiente punto o termina
        if abs(diferencia) >= self.angulo_giro - 0.05:
            self.etapa += 1
            if self.etapa >= len(self.puntos):
                # Se completó la vuelta: detener el robot y apagar el nodo
                self.publicar_comando(0.0, 0.0)
                self.get_logger().info('Vuelta completa realizada correctamente.')
                rclpy.shutdown()
                return
            self.estado = 'AVANCE'

    def publicar_comando(self, velocidad, angulo):
        # Crea y publica un mensaje de velocidad y ángulo de dirección
        msg = AckermannDriveStamped()
        msg.drive.speed = velocidad
        msg.drive.steering_angle = angulo
        self.cmd_pub.publish(msg)

    def obtener_yaw(self, q):
        # Convierte un cuaternión en un ángulo yaw (rotación en Z)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        return math.atan2(siny, cosy)

    def ajustar_angulo(self, ang):
        # Normaliza un ángulo para que esté entre -pi y pi
        while ang > math.pi:
            ang -= 2 * math.pi
        while ang < -math.pi:
            ang += 2 * math.pi
        return ang

def main(args=None):
    rclpy.init(args=args)
    nodo = NavegadorRectangular()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()
