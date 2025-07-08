#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

# Clase que implementa el algoritmo Follow the Gap
class CustomDriver:
    # Constantes del algoritmo
    RADIO_BURBUJA = 140                         # Zona de seguridad alrededor del obstáculo más cercano
    TAM_FILTRO_PREPROC = 3                      # Tamaño del filtro de suavizado para el preprocesamiento del LiDAR
    TAM_FILTRO_MEJOR_PTO = 40                   # Tamaño de la ventana para suavizar al buscar el mejor punto
    DIST_MAX_LIDAR = 30.0                       # Distancia máxima válida de lectura del LiDAR

    UN_GRADO = np.pi / 180                      # 1 grado en radianes
    DIEZ_GRADOS = np.pi / 18                    # 10 grados en radianes
    DIST_MURO = 1.0                             # Distancia mínima a las paredes para activar los "muros invisibles"

    def __init__(self):
        # Variables internas del estado del robot
        self.rad_por_elem = None                # Radianes que representa cada lectura del LiDAR
        self.rangos_fov = None                  # Cantidad de datos dentro del campo de visión

        # Controlador PD
        self.Kp = 0.55
        self.Kd = 0.15
        self.error_anterior = 0.0
        self.error_filtrado = 0.0
        self.alpha = 0.25                       # Factor de suavizado exponencial

    # Filtrado y preprocesamiento del arreglo de distancias del LiDAR
    def preprocess_lidar(self, distancias):
        total_medidas = len(distancias)
        fov_grados = 120
        muestras_fov = int((fov_grados / 360.0) * total_medidas)
        inicio = (total_medidas - muestras_fov) // 2
        fin = inicio + muestras_fov
        datos_proc = np.array(distancias[inicio:fin])

        self.rad_por_elem = (2 * np.pi) / len(distancias)
        self.rangos_fov = len(datos_proc)

        # Suavizado
        datos_proc = np.convolve(datos_proc, np.ones(self.TAM_FILTRO_PREPROC), 'same') / self.TAM_FILTRO_PREPROC
        datos_proc = np.clip(datos_proc, 0, self.DIST_MAX_LIDAR)

        # Muros invisibles: se ignoran los extremos si están muy cerca
        margen_indices = 30
        if len(datos_proc) > 2 * margen_indices:
            izq = datos_proc[:margen_indices]
            der = datos_proc[-margen_indices:]

            datos_proc[:margen_indices][izq < self.DIST_MURO] = 0
            datos_proc[-margen_indices:][der < self.DIST_MURO] = 0

        return datos_proc

    # Encuentra el hueco más grande sin obstáculos
    def find_max_gap(self, espacio_libre):
        enmascarado = np.ma.masked_where(espacio_libre == 0, espacio_libre)
        segmentos = np.ma.notmasked_contiguous(enmascarado)
        if not segmentos:
            return 0, len(espacio_libre) - 1
        mejor_segmento = max(segmentos, key=lambda seg: seg.stop - seg.start)
        return mejor_segmento.start, mejor_segmento.stop

    # Encuentra el mejor punto dentro del hueco más grande
    def find_best_point(self, inicio, fin, datos):
        centro = (inicio + fin) // 2
        hueco_prom = np.convolve(datos[inicio:fin], np.ones(self.TAM_FILTRO_MEJOR_PTO), 'same') / self.TAM_FILTRO_MEJOR_PTO
        idx_mas_lejano = hueco_prom.argmax()
        mejor_indice = inicio + idx_mas_lejano
        return int(0.4 * centro + 0.6 * mejor_indice)

    # Aplica un controlador PD para determinar el ángulo de giro
    def get_angle(self, indice, tam):
        centro = tam / 2
        error_bruto = (indice - centro) * self.rad_por_elem
        self.error_filtrado = self.alpha * error_bruto + (1 - self.alpha) * self.error_filtrado
        derivada = self.error_filtrado - self.error_anterior
        self.error_anterior = self.error_filtrado
        return self.Kp * self.error_filtrado + self.Kd * derivada

    # Lógica principal que determina velocidad y ángulo de dirección
    def process_lidar(self, distancias):
        datos_proc = self.preprocess_lidar(distancias)

        centro = len(datos_proc) // 2
        dist_frente = datos_proc[centro]

        # Burbuja de seguridad alrededor del obstáculo más cercano
        indice_cercano = datos_proc.argmin()
        min_i = max(0, indice_cercano - self.RADIO_BURBUJA)
        max_i = min(len(datos_proc) - 1, indice_cercano + self.RADIO_BURBUJA)
        datos_proc[min_i:max_i] = 0

        # Encuentra mejor hueco y punto
        inicio_gap, fin_gap = self.find_max_gap(datos_proc)
        mejor_punto = self.find_best_point(inicio_gap, fin_gap, datos_proc)
        angulo = self.get_angle(mejor_punto, self.rangos_fov)

        # Lógica de velocidad según la geometría del entorno
        curva_suave = (dist_frente < 10.0 or abs(angulo) > self.DIEZ_GRADOS)
        curva_ajustada = (dist_frente < 5.0 and abs(angulo) > self.DIEZ_GRADOS)

        if curva_ajustada:
            velocidad = 5.0
        elif curva_suave:
            velocidad = 8.0
        elif abs(angulo) < self.UN_GRADO:
            velocidad = 12.0
        else:
            velocidad = 10.0

        return velocidad, angulo

# Nodo ROS que ejecuta el algoritmo y publica comandos de velocidad
class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.driver = CustomDriver()
        self.get_logger().info("Follow The Gap ")

    # Callback que procesa cada frame del LiDAR
    def lidar_callback(self, msg):
        velocidad, angulo = self.driver.process_lidar(msg.ranges)
        mensaje_drive = AckermannDriveStamped()
        mensaje_drive.drive.speed = float(velocidad)
        mensaje_drive.drive.steering_angle = float(angulo)
        self.publisher.publish(mensaje_drive)
        self.get_logger().info(f"Velocidad: {velocidad:.2f} m/s | Ángulo: {np.degrees(angulo):.2f}°")

# Función principal
def main(args=None):
    rclpy.init(args=args)
    nodo = FollowTheGapNode()
    rclpy.spin(nodo)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

